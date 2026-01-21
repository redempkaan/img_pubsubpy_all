package com.example.barcodedetectoredge.viewmodel

import android.graphics.Bitmap
import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import com.example.barcodedetectoredge.model.BarcodeResult



class MainViewModel : ViewModel() {

    private val _currentFrame = MutableLiveData<Bitmap>()
    val currentFrame: LiveData<Bitmap> = _currentFrame

    private val _barcodeText = MutableLiveData<String>()
    val barcodeText: LiveData<String> = _barcodeText

    fun updateFrame(bitmap: Bitmap) {
        _currentFrame.postValue(bitmap)
    }

    fun updateBarcode(result: String) {
        _barcodeText.postValue(
            result ?: "No barcode"
        )
    }
}