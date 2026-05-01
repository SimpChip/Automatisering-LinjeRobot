

// ── WiFi + routes ─────────────────────────────────────────────────
void startWiFi() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password, 6);
  delay(400);
  WiFi.setSleep(false);

  server.on("/",            handleRoot);
  server.on("/data",        handleData);
  server.on("/following",   handleFollowing);
  server.on("/drive",       handleDrive);
  server.on("/recalibrate", handleRecalibrate);
  server.on("/reset",       handleReset);
  server.on("/setpid",      handleSetPID); 
  server.on("/metrics",     handleMetrics);  
  server.begin();

  int forsok = 0;
  while (WiFi.softAPIP().toString() == "0.0.0.0") {
    delay(500);
    if (++forsok > 20) ESP.restart();
  }
}

// ── OTA ───────────────────────────────────────────────────────────
void startOTA() {
  ArduinoOTA.setHostname("linefollower");
  ArduinoOTA.setPassword("ota1234");
  ArduinoOTA.onStart([]()           { digitalWrite(LED_RED, LOW);  });
  ArduinoOTA.onEnd([]()             { digitalWrite(LED_RED, HIGH); });
  ArduinoOTA.onError([](ota_error_t) { ESP.restart(); });
  ArduinoOTA.begin();
}
