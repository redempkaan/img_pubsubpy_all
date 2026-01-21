package com.example.barcodedetectoredge.di

import com.example.barcodedetectoredge.model.BarcodeResult
import android.graphics.Bitmap



interface BarcodeScanner {
    fun scan(frameId: Int, bitmap: Bitmap, onResult: (BarcodeResult?) -> Unit)
}