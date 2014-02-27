cc1101mod
=========

An attempt to create linux driver for cc1101 transceiver. Ideally this should be ieee802154 device driver, so I can connect FreeRTOS-based sensor node via linux box to the Internet.

Currently I am testing it on rasbian, thus in order to make it work as a loadable module spidev should be disabled first.
