package com.example.barcodedetectoredge.service.thread

import java.io.BufferedInputStream
import java.io.DataInputStream
import java.net.ServerSocket
import android.util.Log
import com.example.barcodedetectoredge.model.FramePacket
import java.util.concurrent.ArrayBlockingQueue


class FrameReceiverThread(
    private val frameQueue: ArrayBlockingQueue<FramePacket>,
    private val listenPort: Int
) : Thread() {

    override fun run() {

        Log.i("FrameReceiver", "Creating socket")

        val serverSocket = ServerSocket(listenPort)
        val socket = serverSocket.accept()
        Log.i("FrameReceiver", "Connection accepted")

        val input = DataInputStream(BufferedInputStream(socket.getInputStream()))

        while (true) {
            val frameId = input.readInt()
            val length = input.readInt()

            val jpegBytes = ByteArray(length)
            input.readFully(jpegBytes)

            val temp_t = System.nanoTime()


            if (frameQueue.remainingCapacity() == 0) {
                frameQueue.poll()
            }
            frameQueue.offer(FramePacket(frameId, jpegBytes, temp_t))
        }
    }

}