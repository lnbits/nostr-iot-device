#include <Arduino.h>
#include <NostrEvent.h>
#include <NostrRelayManager.h>
#include <NostrRequestOptions.h>

NostrEvent nostr;

void setup() {
  Serial.begin(115200);
  Serial.println("boot");

  String payload = "[\"EVENT\",\"jjvnSc9U7G7LxhanqEkDCzvxou0ittTx1IZ0zyUCOvKsp8gZLrwnquJa0ReYLaQP\",{\"content\":\"xvev9VfN+ojOpBFbh3gzdA==?iv=he9XYOSmQUaCSNikG+DHVA==\",\"created_at\":1698244574,\"id\":\"2500337440f39d09eda8e7cc61d329c05d5e1889905e6a7279b67df908096fe5\",\"kind\":4,\"pubkey\":\"d58d5dc2abdef2195532b0940d56bc44c693b48084bf11d0bb70035510c9e6b5\",\"sig\":\"8c8db50e4680fcdf299c684fc0761b1b025406108d4eb3b921f0fdcf97a4f0d00cb29cc248a26ee4dd1920de0e2037f944ab1763b3db0a62fbe2055902710a48\",\"tags\":[[\"p\",\"22defd21ef1187806b54033e9d657d4430d98efaebd1289bb24b82224b80c7b4\"]]}]";
  String message = nostr.decryptDm("c63fbf2c708b8dcd9049ca61f01b48e9b19d023c3363fd2797ee8842dc48c45e", payload);
  Serial.println(message);

}

void loop() {}