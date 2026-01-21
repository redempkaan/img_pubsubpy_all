package com.example.barcodedetectoredge.service.request

data class Topics(
    val subscribe: Map<String, TopicInfo>,
    val publish: Map<String, TopicInfo>
)
