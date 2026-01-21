package com.example.barcodedetectoredge.service.thread

import com.example.barcodedetectoredge.model.BarcodeResult
import java.util.concurrent.ArrayBlockingQueue
import java.io.DataOutputStream
import java.net.Socket
import org.json.JSONObject


class ResultSendThread(
    private val resultQueue: ArrayBlockingQueue<BarcodeResult>,
    private val droneIp: String,
    private val dronePort: Int
) : Thread() {

    override fun run() {
        val socket = Socket(droneIp, dronePort)
        val output = DataOutputStream(socket.getOutputStream())

        while (true) {
            try {
                val packet = resultQueue.take()


                val json = JSONObject().apply {
                    put("frame_id", packet.frameId)
                    put("barcode", packet.value ?: JSONObject.NULL)
                    put("type", packet.type ?: JSONObject.NULL)
                }

                val jsonBytes = json.toString().toByteArray(Charsets.UTF_8)

                output.writeInt(packet.frameId)
                output.writeInt(jsonBytes.size)
                output.write(jsonBytes)
                output.flush()

            } catch (e: Exception) {
                e.printStackTrace()
                break
            }
        }
    }
}