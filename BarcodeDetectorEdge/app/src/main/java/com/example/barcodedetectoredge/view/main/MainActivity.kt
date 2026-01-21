package com.example.barcodedetectoredge.view.main

import android.os.Bundle
import androidx.appcompat.app.AppCompatActivity
import com.example.barcodedetectoredge.R
import com.example.barcodedetectoredge.service.request.registerToSignaling
import android.widget.ImageView
import android.widget.TextView
import com.example.barcodedetectoredge.service.thread.FrameReceiverThread
import com.example.barcodedetectoredge.model.FramePacket
import java.util.concurrent.ArrayBlockingQueue
import com.example.barcodedetectoredge.model.BarcodeResult
import com.example.barcodedetectoredge.viewmodel.MainViewModel
import androidx.lifecycle.ViewModelProvider
import com.example.barcodedetectoredge.service.thread.ScanThread
import com.example.barcodedetectoredge.di.MlkitScanner
import com.example.barcodedetectoredge.service.thread.ResultSendThread
import android.util.Log
import com.example.barcodedetectoredge.utils.getZeroTierIp
import kotlin.system.exitProcess







val imgListenPort = 8001
val resultSendPort = 7003





class MainActivity : AppCompatActivity() {

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        setContentView(R.layout.activity_main)

        val viewModel = ViewModelProvider(this)[MainViewModel::class.java]


        val imageView = findViewById<ImageView>(R.id.imageView2)
        val textView = findViewById<TextView>(R.id.barcodeText2)


        val frameQueue = ArrayBlockingQueue<FramePacket>(5)
        val resultQueue = ArrayBlockingQueue<BarcodeResult>(5)

        viewModel.barcodeText.observe(this) { result ->
            if (result != "None") {
                textView.text = result
            } else {
                textView.text = "No barcode"
            }
        }

        viewModel.currentFrame.observe(this) { bitmap ->
            imageView.setImageBitmap(bitmap)
        }

        val zerotierIp = getZeroTierIp()

        if (zerotierIp == null) {
            Log.e("ZT", "ZeroTier IP not found")
            exitProcess(-1)
        } else {
            Log.i("ZT", "ZeroTier IP = $zerotierIp")
        }

        registerToSignaling(
            name = "edge_barcode_01",
            ip = zerotierIp!!,
            imgListenPort = imgListenPort,
            resultSendPort = resultSendPort
        )


        val frameReceiver = FrameReceiverThread(
            frameQueue = frameQueue,
            listenPort = imgListenPort
        )

        val scanner = ScanThread(
            frameQueue = frameQueue,
            resultQueue = resultQueue,
            scanner = MlkitScanner(),
            viewModel = viewModel
        )


        val sender = ResultSendThread(
            droneIp = "172.27.122.10",
            dronePort = 8003,
            resultQueue = resultQueue)




        frameReceiver.start()
        scanner.start()
        sender.start()



    }
}