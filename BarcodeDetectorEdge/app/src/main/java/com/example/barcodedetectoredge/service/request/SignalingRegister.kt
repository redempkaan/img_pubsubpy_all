package com.example.barcodedetectoredge.service.request

import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response
import android.util.Log




fun registerToSignaling(name: String, ip: String, imgListenPort: Int, resultSendPort: Int) {

    val subscribeTopics = mapOf(
        "/img/raw" to TopicInfo(
            port = imgListenPort,
            protocol = "tcp",
            datatype = "jpeg"
        )
    )

    val publishTopics = mapOf(
        "/barcode/result" to TopicInfo(
            port = resultSendPort,
            protocol = "tcp",
            datatype = "json"
        )
    )

    val request = RegisterRequest(
        name = name,
        ip = ip,
        node_type = "edge",
        topics = Topics(
            subscribe = subscribeTopics,
            publish = publishTopics
        )
    )

    SignalingClient.api.registerNode(request)
        .enqueue(object : Callback<Void> {

            override fun onResponse(
                call: Call<Void>,
                response: Response<Void>
            ) {
                if (response.isSuccessful) {
                    Log.i("SIGNAL", "Register successful: $name")
                } else {
                    Log.e("SIGNAL", "Register failed: ${response.code()}")
                }
            }

            override fun onFailure(call: Call<Void>, t: Throwable) {
                Log.e("SIGNAL", "Register error", t)
            }
        })
}
