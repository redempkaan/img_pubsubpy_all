package com.example.barcodedetectoredge.service.request

data class TopicEndpoint(
    val node: String,
    val ip: String,
    val port: Int,
    val protocol: String,
    val datatype: String
)

data class TopicResponse(
    val topic: String,
    val publishers: List<TopicEndpoint>,
    val subscribers: List<TopicEndpoint>
)