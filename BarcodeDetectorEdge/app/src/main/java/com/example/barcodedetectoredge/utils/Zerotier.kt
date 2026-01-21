package com.example.barcodedetectoredge.utils

import java.net.Inet4Address
import java.net.NetworkInterface


fun getZeroTierIp(): String? {
    val interfaces = NetworkInterface.getNetworkInterfaces()

    for (iface in interfaces) {
        if (!iface.isUp || iface.isLoopback) continue

        // ZeroTier interface isimleri
        if (iface.name.startsWith("zt") || iface.name.startsWith("tun")) {
            for (addr in iface.inetAddresses) {
                if (!addr.isLoopbackAddress && addr is Inet4Address) {
                    return addr.hostAddress
                }
            }
        }
    }
    return null
}