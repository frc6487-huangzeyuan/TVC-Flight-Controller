#include "PrimaryDebug.h"
// 串口 UART1 统一由 sensorTask 访问；Serial 继续用于电脑 USB 通信和网页协议。
HardwareSerial primarySerial(1);
PrimaryUart::Receiver primaryReceiver;
PrimaryUart::Parser primaryParser;
uint32_t previousRemoteErrors=0;
bool haveRemoteErrors=false;
PrimaryUart::Response primaryDebugFrame;
PrimaryUart::Result primaryDebugResult=PrimaryUart::REJECTED;
bool primaryDebugPending=false;
uint32_t primaryDebugSeenAt=0;

void beginPrimaryUart(){
  primarySerial.setRxBufferSize(256);
  primarySerial.begin(PRIMARY_UART_BAUD,SERIAL_8N1,PRIMARY_UART_RX,PRIMARY_UART_TX);
  primaryReceiver.begin(esp_random());
}
bool pollPrimaryUart(PrimaryUart::Sample &sample){
  bool accepted=false;const uint8_t *packet=nullptr;
  for(int n=0;n<256&&primarySerial.available();++n){
    if(!primaryParser.push(uint8_t(primarySerial.read()),2,packet))continue;
    const auto response=PrimaryUart::response(packet);PrimaryUart::Sample candidate;
    const auto result=primaryReceiver.accept(response,esp_timer_get_time(),candidate);
    xSemaphoreTake(stateLock,portMAX_DELAY);
    primaryDebugSeenAt=millis();
    if(response.id==1||response.id%100==0){primaryDebugFrame=response;primaryDebugResult=result;primaryDebugPending=true;}
    xSemaphoreGive(stateLock);
    if(result==PrimaryUart::REJECTED)continue;
    xSemaphoreTake(stateLock,portMAX_DELAY);
    // 短暂报告未找到传感器时，不丢弃此前的有效样本；历史窗口结束再失效。
    primaryImu.present=response.status!=PrimaryUart::NOT_FOUND ||
      (primaryImu.count && primaryImu.t && uint64_t(esp_timer_get_time())>=primaryImu.t &&
       uint64_t(esp_timer_get_time())-primaryImu.t<IMU_TIMEOUT_US);
    const uint32_t errors=(!haveRemoteErrors||candidate.restarted)?response.readErrors:response.readErrors-previousRemoteErrors;
    primaryImu.readErrors+=errors;previousRemoteErrors=response.readErrors;haveRemoteErrors=true;
    primaryRemoteBoot=response.boot;
    if(candidate.restarted){
      primaryImu.valid=false;primaryImu.t=0;
      // 只有采集板这一路正在作为控制源时，重启才需要停止输出并报告故障；
      // 未被选中时它只是校验数据，不影响正在使用的 IMU2。
      if(controlSource==SOURCE_IMU1){
        worldReady=false;calibrating=calibrated=imu.calibrated=false;attitude.invalidate();correctionActive=false;
        latchFault("IMU_RESTART");
      }
    }
    xSemaphoreGive(stateLock);
    if(result==PrimaryUart::ACCEPTED){sample=candidate;accepted=true;}
  }
  uint8_t request[PrimaryUart::REQUEST_BYTES];
  if(primarySerial.availableForWrite()>=int(sizeof(request))&&primaryReceiver.makeRequest(esp_timer_get_time(),request))
    primarySerial.write(request,sizeof(request));
  xSemaphoreTake(stateLock,portMAX_DELAY);
  primaryReplies=primaryReceiver.replies;primaryBadFrames=primaryParser.badFrames;
  primaryRejected=primaryReceiver.rejected;primaryTimeouts=primaryReceiver.timeouts;
  primaryImu.errors=primaryImu.readErrors+primaryBadFrames+primaryRejected;
  xSemaphoreGive(stateLock);
  return accepted;
}

// 只有 loop() 向电脑 Serial 写数据，保证调试日志和 JSON 帧逐条完整发送。
void printPrimaryDebug(){
  if(!Serial)return;
  static uint32_t idleAt=0;
  PrimaryUart::Response frame;PrimaryUart::Result result;bool pending;
  uint32_t seen,frames,crc,rejected,timeouts;
  xSemaphoreTake(stateLock,portMAX_DELAY);
  frame=primaryDebugFrame;result=primaryDebugResult;pending=primaryDebugPending;seen=primaryDebugSeenAt;
  frames=primaryReplies;crc=primaryBadFrames;rejected=primaryRejected;timeouts=primaryTimeouts;
  xSemaphoreGive(stateLock);
  char line[160];int n=0;
  if(pending){
    const char *reason=result==PrimaryUart::ACCEPTED?"OK":result==PrimaryUart::DUPLICATE?"DUP":result==PrimaryUart::STATUS_ONLY?"WAIT":"REJECT";
    n=formatPrimaryDebug(line,sizeof(line),"S3_RX",reason,frame);
  }else if(millis()-seen>=1000&&millis()-idleAt>=1000){
    n=snprintf(line,sizeof(line),"#IMU1 S3_WAIT replies=%lu crc=%lu reject=%lu timeout=%lu\n",(unsigned long)frames,(unsigned long)crc,(unsigned long)rejected,(unsigned long)timeouts);
  }
  if(n<=0||n>=int(sizeof(line))||Serial.availableForWrite()<n)return;
  Serial.write(reinterpret_cast<const uint8_t*>(line),size_t(n));
  if(pending){
    xSemaphoreTake(stateLock,portMAX_DELAY);
    if(primaryDebugFrame.id==frame.id&&primaryDebugFrame.session==frame.session)primaryDebugPending=false;
    xSemaphoreGive(stateLock);
  }else idleAt=millis();
}
