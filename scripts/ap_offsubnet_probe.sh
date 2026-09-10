#!/usr/bin/env bash
# Does the Go2, in AP mode, answer a peer whose address is NOT on its own subnet?
#
# This is the load-bearing question for "can anything off the hotspot drive an
# AP-mode dog" — the answer is why the AP path is inverted (the laptop holds the
# WebRTC link and feeds EC2's ROS graph; see scripts/go2_ap_bridge.py) rather than
# tunnelled. It is worth 30 seconds before building anything on top of AP mode.
#
# In AP mode the dog is the access point, the DHCP server (192.168.12.1) and its own
# gateway, with no uplink. If it has no route off 192.168.12.0/24 then:
#
#   - a plain routed tunnel cannot work at all, not even for signaling: the SYN
#     arrives from off-subnet and the SYN-ACK has nowhere to go
#   - the peer must NAT, so the dog only ever sees a 192.168.12.x source
#   - and ICE still fails afterwards, because a remote peer ADVERTISES its own
#     unroutable address in the SDP regardless of what the packet headers say
#
# If instead the dog DOES answer an off-subnet source, it has a usable default
# route, and more options open up.
#
# The trick: send from an address the dog cannot route to, and see if a reply comes
# back. `ping -S` and `nc -s` both set the source address without any NAT involved.
#
# Run it on the Mac, joined to the dog's hotspot AND with ethernet plugged in:
#     ./scripts/ap_offsubnet_probe.sh
#
# Exit 0 = probe ran and reported; read the VERDICT. Exit 2 = preconditions unmet.
set -uo pipefail

DOG="${DOG_IP:-192.168.12.1}"
PORT="${SIGNAL_PORT:-9991}"

echo "=== Go2 AP off-subnet routing probe -> $DOG:$PORT ==="
echo

# --- Find the two interfaces ------------------------------------------------
# The AP interface is whichever one holds a 192.168.12.x address. The off-subnet
# source is any other interface with an address — normally the USB ethernet.
ap_if="" ap_ip="" off_if="" off_ip=""
for i in $(ifconfig -l); do
  a=$(ipconfig getifaddr "$i" 2>/dev/null) || continue
  [ -n "$a" ] || continue
  case "$a" in
    192.168.12.*) ap_if="$i"; ap_ip="$a" ;;
    127.*|169.254.*) ;;
    *) if [ -z "$off_if" ]; then off_if="$i"; off_ip="$a"; fi ;;
  esac
done

if [ -z "$ap_ip" ]; then
  echo "[!] No interface has a 192.168.12.x address."
  echo "    Join the dog's hotspot first (Wi-Fi), then re-run."
  exit 2
fi
echo "[i] AP interface:        $ap_if  $ap_ip   (on the dog's subnet)"

if [ -z "$off_ip" ]; then
  echo "[!] No second interface with an address — nothing to source an off-subnet"
  echo "    packet from. Plug in the USB ethernet (en6/en8) and re-run."
  exit 2
fi
echo "[i] Off-subnet interface: $off_if  $off_ip  (what EC2 would look like)"
echo

# --- Control: same-subnet source must work ----------------------------------
# If this fails the dog is unreachable for an unrelated reason and the real probe
# below would be meaningless.
echo "--- CONTROL: from the AP address ($ap_ip), which the dog can reach ---"
if ping -c 2 -t 3 -S "$ap_ip" "$DOG" >/dev/null 2>&1; then
  echo "[ok] ICMP reply     (dog is up and answering on its own subnet)"
  control_icmp=yes
else
  echo "[!!] no ICMP reply — some Go2 firmware does not answer ping; continuing"
  control_icmp=no
fi
if nc -z -w 3 -s "$ap_ip" "$DOG" "$PORT" 2>/dev/null; then
  echo "[ok] TCP :$PORT open  (signaling reachable, as expected)"
else
  echo "[!] TCP :$PORT did NOT open from the dog's own subnet."
  echo "    Something else is wrong (dog asleep? already-connected peer?)."
  echo "    Fix that before trusting the probe below."
  exit 2
fi
echo

# --- The real probe: off-subnet source --------------------------------------
# A TCP connect is the meaningful test: it needs a full round trip, so it only
# completes if the dog can route a SYN-ACK back to an address off its subnet.
echo "--- PROBE: from the ethernet address ($off_ip), which the dog has no route to ---"
off_icmp=no
if ping -c 2 -t 3 -S "$off_ip" "$DOG" >/dev/null 2>&1; then
  echo "[ok] ICMP reply to an off-subnet source"
  off_icmp=yes
else
  echo "[--] no ICMP reply to an off-subnet source"
fi

off_tcp=no
if nc -z -w 5 -s "$off_ip" "$DOG" "$PORT" 2>/dev/null; then
  echo "[ok] TCP :$PORT COMPLETED from an off-subnet source"
  off_tcp=yes
else
  echo "[--] TCP :$PORT did not complete from an off-subnet source"
fi
echo

# --- Verdict ----------------------------------------------------------------
echo "=== VERDICT ==="
if [ "$off_tcp" = yes ]; then
  cat <<'MSG'
The dog CAN talk to an off-subnet peer. It has a usable route (probably a default
gateway pointing at whatever DHCP client is present, or it simply replies via its
only interface).

This is the GOOD outcome. It means:
  - a routed tunnel to the dog may work with no NAT at all
  - ICE may still fail, because a remote peer advertises its own address in the SDP
    and the dog has to reach THAT specific address, not just answer our packets

So this alone does not make a remote peer workable — but it does mean the routing
half of the problem is not the blocker.
MSG
else
  cat <<'MSG'
The dog CANNOT talk to an off-subnet peer — TCP from an off-subnet source never
completes, so it has no route back off 192.168.12.0/24.

This settles two things:
  - a routed-only tunnel is dead on arrival; the peer MUST NAT so the dog only
    ever sees a 192.168.12.x source
  - and NAT alone still is not enough, because a remote peer advertises its own
    unroutable address in the SDP. The dog acts on the SDP, not on packet headers.

Which is why the AP path is inverted: whatever is ON the hotspot ($ap_ip) holds the
WebRTC link, and the ROS data is forwarded from there. See scripts/go2_ap_bridge.py
and `make ap-bridge`.
MSG
fi

if [ "$control_icmp" = yes ] && [ "$off_icmp" = no ] && [ "$off_tcp" = no ]; then
  echo
  echo "[i] Clean signal: ICMP worked from the AP address and not from the other,"
  echo "    which is the routing asymmetry itself rather than a filtered port."
fi
