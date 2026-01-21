package com.example.barcodedetectoredge.model

data class FramePacket(
    val frameId: Int,
    val payload: ByteArray,
    val timestamp: Long
) {
    val length: Int
        get() = payload.size
}
