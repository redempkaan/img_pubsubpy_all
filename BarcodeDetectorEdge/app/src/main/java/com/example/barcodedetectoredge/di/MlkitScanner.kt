package com.example.barcodedetectoredge.di

import com.google.mlkit.vision.barcode.common.Barcode
import com.google.mlkit.vision.barcode.BarcodeScanning
import com.google.mlkit.vision.barcode.BarcodeScannerOptions
import com.google.mlkit.vision.common.InputImage
import com.example.barcodedetectoredge.model.BarcodeResult
import android.graphics.Bitmap


class MlkitScanner : BarcodeScanner {

    private val options = BarcodeScannerOptions.Builder()
        .setBarcodeFormats(
            Barcode.FORMAT_QR_CODE,
            Barcode.FORMAT_AZTEC,
            Barcode.FORMAT_CODE_128,
            Barcode.FORMAT_CODE_39,
            Barcode.FORMAT_EAN_13,
            Barcode.FORMAT_EAN_8
        )
        .build()

    private val scanner = BarcodeScanning.getClient(options)

    override fun scan(frameId: Int, bitmap: Bitmap, onResult: (BarcodeResult?) -> Unit) {
        val image = InputImage.fromBitmap(bitmap, 0)

        scanner.process(image).addOnSuccessListener{
            barcodes ->
                if (barcodes.isNotEmpty()) {
                    val b = barcodes.first()
                    onResult(BarcodeResult(frameId = frameId, type = formatToString(b.format) ?: "None", value = b.rawValue ?: "None"))
                }
                else {
                    onResult(BarcodeResult(frameId = frameId, type = "None", value = "None"))
                }
            }
            .addOnFailureListener {
                onResult(BarcodeResult(frameId = frameId, type = "None", value = "None"))
            }
    }

    fun formatToString(format: Int): String {
        return when (format) {
            Barcode.FORMAT_QR_CODE -> "QR_CODE"
            Barcode.FORMAT_CODE_128 -> "CODE_128"
            Barcode.FORMAT_CODE_39 -> "CODE_39"
            Barcode.FORMAT_EAN_13 -> "EAN_13"
            Barcode.FORMAT_EAN_8 -> "EAN_8"
            Barcode.FORMAT_AZTEC -> "AZTEC"
            Barcode.FORMAT_DATA_MATRIX -> "DATA_MATRIX"
            Barcode.FORMAT_PDF417 -> "PDF_417"
            else -> "UNKNOWN"
        }
    }
}