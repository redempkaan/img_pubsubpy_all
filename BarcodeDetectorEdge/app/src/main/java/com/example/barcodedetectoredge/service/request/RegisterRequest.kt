package com.example.barcodedetectoredge.service.request

data class RegisterRequest(
    val name: String,
    val ip: String,
    val node_type: String,
    val topics: Topics
)