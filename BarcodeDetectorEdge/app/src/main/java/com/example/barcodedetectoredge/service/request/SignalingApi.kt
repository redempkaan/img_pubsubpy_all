package com.example.barcodedetectoredge.service.request

import retrofit2.Call
import retrofit2.http.Body
import retrofit2.http.POST
import retrofit2.http.GET
import retrofit2.http.Path


interface SignalingApi {

    @POST("/register")
    fun registerNode(
        @Body request: RegisterRequest
    ): Call<Void>

    @GET("/node/{name}")
    fun getNode(
        @Path("name") name: String
    ): Call<Map<String, Any>>

    @GET("/topic")
    fun getTopic(
        @retrofit2.http.Query("name") topic: String
    ): Call<TopicResponse>
}