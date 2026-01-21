package com.example.barcodedetectoredge.service.request

import retrofit2.Retrofit
import retrofit2.converter.gson.GsonConverterFactory

object SignalingClient {
    private const val BASE_URL = "http://13.62.99.235:80"

    val api: SignalingApi by lazy {
        Retrofit.Builder()
            .baseUrl(BASE_URL)
            .addConverterFactory(GsonConverterFactory.create())
            .build()
            .create(SignalingApi::class.java)
        }
    }
