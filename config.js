export const addressesAndFiles = [
  {
    address: "0x1000",
    fileName: "nostrasia-iot.ino.bootloader.bin",
  },
  {
    address: "0x8000",
    fileName: "nostrasia-iot.ino.partitions.bin",
  },
  {
    address: "0xE000",
    fileName: "boot_app0.bin",
  },
  {
    address: "0x10000",
    fileName: "nostrasia-iot.ino.bin",
  },
];

export const configPath = "elements.json";

export const elements = [
  {
    name: "config_ssid",
    value: "",
    label: "WiFi SSID",
    type: "text",
  },
  {
    name: "config_password",
    value: "",
    label: "WiFi password",
    type: "password",
  },
  {
    name: "config_pk",
    value: "",
    label: "Nostr private key (hex)",
    type: "password",
  }
];
