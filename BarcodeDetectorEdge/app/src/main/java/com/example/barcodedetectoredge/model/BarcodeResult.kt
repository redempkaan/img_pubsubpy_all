package com.example.barcodedetectoredge.model

data class BarcodeResult(
    val frameId: Int,
    val type: String,
    val value: String
)
