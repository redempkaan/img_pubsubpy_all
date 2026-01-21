package com.example.barcodedetectoredge.service.thread

import android.graphics.BitmapFactory
import java.util.concurrent.BlockingQueue
import com.example.barcodedetectoredge.model.FramePacket
import com.example.barcodedetectoredge.di.BarcodeScanner
import com.example.barcodedetectoredge.model.BarcodeResult
import com.example.barcodedetectoredge.viewmodel.MainViewModel
import android.util.Log



class ScanThread(
    private val frameQueue: BlockingQueue<FramePacket>,
    private val resultQueue: BlockingQueue<BarcodeResult>,
    private val scanner: BarcodeScanner,
    private val viewModel: MainViewModel
    ) : Thread() {

    override fun run() {
        while (true) {
            val framePacket = frameQueue.take()

            val bitmap = BitmapFactory.decodeByteArray(
                framePacket.payload, 0, framePacket.payload.size
            ) ?: continue

            viewModel.updateFrame(bitmap)

            scanner.scan(framePacket.frameId, bitmap) { result ->
                viewModel.updateBarcode(result!!.value)
                resultQueue.put(result)
                Log.d("Frame[${framePacket.frameId}]", "Result latency: ${(System.nanoTime() - framePacket.timestamp) / 1_000_000.0}")
            }
        }
    }
}