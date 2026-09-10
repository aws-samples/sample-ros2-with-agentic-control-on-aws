# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause
# Modified by Amazon.com, Inc. or its affiliates.

"""
Data decoder for Go2 WebRTC binary messages.
Handles decoding of compressed LiDAR data and other binary messages from WebRTC.
"""

import json
import struct
import logging
from typing import Optional, Dict, Any, Union


# Defer importing `..sensors.lidar_decoder` until the first lidar
# packet arrives. Importing it eagerly drags in `wasmtime` plus the
# numpy/ctypes machinery at module load — which (in this Python
# environment) installs handlers/state that wedge aiortc's DTLS task
# at "ICE completed -> peer connecting" forever. We've reproduced
# this minimally by toggling just this import.
def _load_original_lidar_decoder():
    try:
        from ..sensors.lidar_decoder import LidarDecoder as OriginalLidarDecoder
        return OriginalLidarDecoder
    except ImportError:
        return None


OriginalLidarDecoder = None  # populated lazily on first decode call


logger = logging.getLogger(__name__)


class DataDecodingError(Exception):
    """Custom exception for data decoding errors"""
    pass


class WebRTCDataDecoder:
    """Decoder for WebRTC binary data messages"""
    
    def __init__(self, enable_lidar_decoding: bool = True):
        """
        Initialize data decoder.
        
        Args:
            enable_lidar_decoding: Whether to decode LiDAR data or keep it compressed
        """
        # Defer wasmtime instantiation until the first lidar packet
        # arrives — initializing wasmtime eagerly breaks aiortc's DTLS.
        self.enable_lidar_decoding = enable_lidar_decoding
        self._lidar_decoder = None

    def _ensure_lidar_decoder(self):
        if self._lidar_decoder is None and self.enable_lidar_decoding:
            try:
                cls = _load_original_lidar_decoder()
                if cls is not None:
                    self._lidar_decoder = cls()
                    logger.info("Using LidarDecoder")
                else:
                    self.enable_lidar_decoding = False
            except Exception as e:
                logger.warning(f"Failed to initialize LiDAR decoder: {e}")
                self.enable_lidar_decoding = False
        return self._lidar_decoder
    
    def decode_array_buffer(self, buffer: bytes) -> Optional[Dict[str, Any]]:
        """
        Decode binary array buffer from WebRTC data channel.
        
        The buffer format is:
        - First 2 bytes: Length of JSON segment (little-endian uint16)
        - Next 2 bytes: Reserved/padding
        - Next <length> bytes: JSON metadata
        - Remaining bytes: Compressed data
        
        Args:
            buffer: Binary data from WebRTC data channel
            
        Returns:
            Dictionary containing decoded data or None if decoding fails
        """
        if not isinstance(buffer, bytes):
            logger.error("Buffer must be bytes type")
            return None
        
        if len(buffer) < 4:
            logger.error("Buffer too short, minimum 4 bytes required")
            return None
        
        try:
            # Extract length of JSON segment (first 2 bytes, little-endian)
            json_length = struct.unpack("<H", buffer[:2])[0]
            
            logger.debug(f"JSON segment length: {json_length}")
            
            # Validate buffer length
            if len(buffer) < 4 + json_length:
                logger.error(f"Buffer too short for JSON segment. Expected {4 + json_length}, got {len(buffer)}")
                return None
            
            # Extract JSON segment (skip 2 bytes header + 2 bytes padding)
            json_segment = buffer[4:4 + json_length]
            
            # Extract compressed data (remaining bytes)
            compressed_data = buffer[4 + json_length:]
            
            # Decode JSON metadata
            try:
                json_str = json_segment.decode("utf-8")
                metadata = json.loads(json_str)
            except (UnicodeDecodeError, json.JSONDecodeError) as e:
                logger.error(f"Failed to decode JSON segment: {e}")
                return None
            
            logger.debug(f"Decoded metadata: {metadata}")
            
            # Add compressed data to result
            result = metadata.copy()
            
            # Decode LiDAR data if enabled and decoder is available
            if self.enable_lidar_decoding and compressed_data and self._ensure_lidar_decoder():
                try:
                    decoded_data = self._decode_lidar_data(compressed_data, metadata)
                    result["decoded_data"] = decoded_data
                    logger.debug("Successfully decoded LiDAR data")
                except Exception as e:
                    logger.warning(f"Failed to decode LiDAR data: {e}")
                    result["compressed_data"] = compressed_data
            else:
                result["compressed_data"] = compressed_data
            
            return result
            
        except struct.error as e:
            logger.error(f"Failed to unpack buffer header: {e}")
            return None
        except Exception as e:
            logger.error(f"Unexpected error decoding array buffer: {e}")
            return None
    
    def _decode_lidar_data(self, compressed_data: bytes, metadata: Dict[str, Any]) -> Dict[str, Any]:
        """
        Decode compressed LiDAR data using WASM decoder.
        
        Args:
            compressed_data: Compressed voxel map data
            metadata: Metadata containing decoding parameters
            
        Returns:
            Dictionary containing decoded LiDAR data
            
        Raises:
            DataDecodingError: If decoding fails
        """
        if not self._lidar_decoder:
            raise DataDecodingError("LiDAR decoder not available")
        
        if not compressed_data:
            raise DataDecodingError("No compressed data to decode")
        
        try:
            # Use the original LidarDecoder interface
            decoded_result = self._lidar_decoder.decode(compressed_data, metadata)
            
            return decoded_result
            
        except Exception as e:
            raise DataDecodingError(f"LiDAR decoding failed: {e}")
    
    def set_lidar_decoding(self, enabled: bool) -> None:
        """
        Enable or disable LiDAR decoding.
        
        Args:
            enabled: Whether to enable LiDAR decoding
        """
        self.enable_lidar_decoding = enabled
        
        if enabled and self._lidar_decoder is None:
            try:
                cls = _load_original_lidar_decoder()
                if cls is not None:
                    self._lidar_decoder = cls()
                else:
                    raise ImportError("LiDAR decoder not available")
            except Exception as e:
                logger.warning(f"Failed to initialize LiDAR decoder: {e}")
                self.enable_lidar_decoding = False


# Global decoder instance for backward compatibility
_global_decoder: Optional[WebRTCDataDecoder] = None


def get_data_decoder(enable_lidar: bool = True) -> WebRTCDataDecoder:
    """
    Get global data decoder instance.
    
    Args:
        enable_lidar: Whether to enable LiDAR decoding
        
    Returns:
        WebRTCDataDecoder instance
    """
    global _global_decoder
    
    if _global_decoder is None or _global_decoder.enable_lidar_decoding != enable_lidar:
        _global_decoder = WebRTCDataDecoder(enable_lidar)
    
    return _global_decoder


# Lazy global decoder. Eagerly instantiating wasmtime at module import
# time installs signal handlers that interfere with aiortc's DTLS task
# (peer state sticks at "connecting" forever after ICE completes). We
# defer the decoder until the first lidar packet arrives — by then the
# WebRTC handshake has already finished.
_global_lidar_decoder: Optional[Any] = None


def _get_global_lidar_decoder():
    global _global_lidar_decoder
    if _global_lidar_decoder is None:
        cls = _load_original_lidar_decoder()
        if cls is not None:
            try:
                _global_lidar_decoder = cls()
            except Exception as e:
                logger.warning(f"Failed to init lidar decoder: {e}")
    return _global_lidar_decoder


# Backward compatibility function
def deal_array_buffer(buffer: bytes, perform_decode: bool = True) -> Optional[Dict[str, Any]]:
    """
    Legacy function for backward compatibility.
    
    Args:
        buffer: Binary data buffer
        perform_decode: Whether to decode LiDAR data
        
    Returns:
        Decoded data dictionary or None
    """
    if not isinstance(buffer, bytes):
        return None
    
    try:
        decoder = _get_global_lidar_decoder() if perform_decode else None
        if decoder and perform_decode:
            import struct
            import json

            length = struct.unpack("H", buffer[:2])[0]
            json_segment = buffer[4: 4 + length]
            compressed_data = buffer[4 + length:]
            json_str = json_segment.decode("utf-8")
            obj = json.loads(json_str)

            if compressed_data:
                decoded_data = decoder.decode(compressed_data, obj['data'])
                obj["decoded_data"] = decoded_data
            else:
                obj["compressed_data"] = compressed_data
            return obj
        else:
            # Fallback to new decoder (also lazily instantiated)
            new_decoder = get_data_decoder(enable_lidar=perform_decode)
            return new_decoder.decode_array_buffer(buffer)
            
    except Exception as e:
        logger.error(f"Error in deal_array_buffer: {e}")
        return None 