#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <EEPROM.h>
#include <Wire.h>
#include <MPU6050_tockn.h>
MPU6050 mpu6050(Wire);

long timer = 0;
long timerWhileOnOff = 0;
bool thoatVongLapRelay = false;

HardwareSerial serialGPS(1); // use UART1
TinyGPSPlus gps;
#define pwkSim7600 5
#define txGPS 32
#define rxGps 35

// các chân của remote
#define RF1 33
#define RF2 25
#define RF3 26
#define RF4 27

#define loa 13  // loa
#define SW420 4 // cam bien rung
#define relay 14

#define gocTeNga 50

#define SimA7600C Serial2
// TX pin->GPIO 16, RX pin->GPIO 17

// Firebase URL-----------------------------------------
const String firebaseUrl = "https://trackingvehicale-default-rtdb.firebaseio.com/User1/GPS";
const String firebaseScret = "vgpWQHHqKUOGcB1N0jwMGcjwqZEdtviSYd6qLG38";

// thiet lap nha mang
char apn[] = "m9-wintel";
char user[] = "";
char pass[] = "";

unsigned long millisMpu6050 = 0;
unsigned long millisTraking = 0;
unsigned long millisRung = 0;
unsigned long millisTaiNan = 0;
unsigned long currentMillis = millis();
// unsigned long lastDataTime = millis();

int flagRF4 = 0;
int flagSW420 = 0;
int flagRelayOn = 0;
int flagRelayOff = 1;
int flagMPU6050 = 0;
int flagTrackingXeMay = 0;
// int flag_accident = 0;
int flagDangKy = 0;

unsigned long long timeTrankingXeMay = 0;

int flagNgatMpu6065 = 0;
int flagRF4_2 = 0;

// cam bien goc nghieng
float gocX, gocY, gocZ; // gia tri goc
float gocXBanDau, gocYBanDau;

// bien gia tri truoc do cua GPS
float previousLatitude = 0.0; // Giá trị khởi tạo, có thể điều chỉnh tùy vào yêu cầu
float previousLongitude = 0.0;

const int tongSoDienThoai = 5;
String phoneNo[tongSoDienThoai] = {"", "", "", "", ""};
int offsetPhone[tongSoDienThoai] = {0, 12, 24, 36, 48};
String tempPhone = "";

String smsStatus;
String senderNumber;
String msg;
String IndexSms;
String latitude, longitude;

// function prototype

void timXe(void); // #1 tim xe
void phatHienVaCham(void);
void batTatXe(void);
void phatHienTeNga(void);
void xuLiTinNhan(String);
void xuLiTinNhanRelay(String);
void extract_sms(String);
// void calibration_mpu6050(void);
void loa_2bip(void);
void trackingXeMayRealtime(void);
void dayDuLieuLenFirebase(String);
void vietDuLieuVaoEEPROM(int, const String *);
String docDuLieuTuEEPROM(int);
void guiTinNhan(String, String);
void guiViTri(String);
void do_action(String);
void do_action_y_n(String);
void canhBaoXeBiDatTrom();


void testBuzz();
void layGocMPU6050();
void testGPS();

void setup()
{

  // cua rf
  // Serial.begin(9600);
  pinMode(RF1, INPUT);
  pinMode(RF2, INPUT);
  pinMode(RF3, INPUT);
  pinMode(RF4, INPUT);
  pinMode(relay, OUTPUT);
  pinMode(loa, OUTPUT);

  // cam bien goc
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  // GPS ----------------------------------------------------------------//

  serialGPS.begin(9600, SERIAL_8N1, rxGps, txGPS); // Cấu hình UART1 cho GPS
  Serial.println("khoi tao GPS serial");

  // sim --------------------------------------------------------------------
  Serial.begin(115200); // config baud rate UART1 for debug
  Serial.println("khoi tao NodeMCU USB serial");
  SimA7600C.begin(115200);
  Serial.println("khoi taoSIMA7600C serial ");
  pinMode(pwkSim7600, OUTPUT);
  digitalWrite(pwkSim7600, HIGH);
  delay(1000);

  // khoi tạo eeprom;
  EEPROM.begin(512);

  // hien thi so đt thoại đã đang kí chưa
  Serial.println("danh sach so dien thoai da duoc dang ki");
  for (int i = 0; i < tongSoDienThoai; i++)
  {
    phoneNo[i] = docDuLieuTuEEPROM(offsetPhone[i]);
    if (phoneNo[i].length() != 12)
    {
      phoneNo[i] = "";
      Serial.println(String(i + 1) + ": empty");
    }
    else
    {
      Serial.println(String(i + 1) + ": " + phoneNo[i]);
    }
  }

  //
  smsStatus = "";
  senderNumber = "";
  msg = "";

  // sim 7600C

  // Đây là một lệnh AT để truy vấn trạng thái của module liên quan đến đăng ký mạng di động.
  SimA7600C.println("AT+CFUN=1");
  delay(1000);
  SimA7600C.println("AT+CEREG=?");
  // Lệnh AT này sử dụng để xóa tin nhắn SMS đã nhận trong bộ nhớ
  SimA7600C.println("AT+CMGD=0,4");
  delay(1000);
  // Lệnh này sử dụng để khôi phục module về trạng thái ban đầu (reset).
  SimA7600C.println("ATZ0"); // will restore the user setting from ME which set by ATE, ATQ, ATV,.....
  delay(1000);
  SimA7600C.println("ATE0"); // tat che do echo
  delay(1000);

  loa_2bip();
  millisMpu6050 = millis();
}

void loop()
{
  /*
  // test rf
  Serial.println("Input RF1 is HIGH");
  Serial.println(digitalRead(INPUT_RF1));
  Serial.println("Input RF2 is HIGH");
  Serial.println(digitalRead(INPUT_RF2));
  Serial.println("Input RF3 is HIGH");
  Serial.println(digitalRead(INPUT_RF3));
  Serial.println("Input RF4 is HIGH");
  Serial.println(digitalRead(INPUT_RF4));
  timXe();
  batTatXe();
  delay(1000);
  //
  */

  /*
  // cam bien goc
  layGocMPU6050();
  //
  */

  /*
  // cam bien rung
  Serial.println("Input cam bien rung");
  Serial.println(digitalRead(SW420));
  delay(1000);
  //
  */

  // GPS
  //  testGPS();
  //  Serial.println("GPS");

  // bat tat cam bien rung, bao dong
  // phatHienVaCham();
  // Serial.println("Input RF4 is HIGH");
  // Serial.println(digitalRead(INPUT_RF4));
  // Serial.println("Input cam bien rung");
  // Serial.println(digitalRead(SW420));
  // delay(1000);

  //---------------------------------------------------
  // test lien ket voi firebaser----------------------
  // trackingXeMayRealtime();
  //----------------------------------------------------

  //--------------------------------------------------

  // test gưi vị tri dên điện thoại
  // String phoneNumber = "0703487023";
  // guiViTri(phoneNumber);

  //
  /*code chính  */
  // digitalWrite(PWK_Sim, LOW);
  timXe(); // tìm xe

  phatHienVaCham(); // phat hien va cham, rung lac
  Serial.println("\nInput RF4 is HIGH");
  Serial.println(digitalRead(RF4));
  Serial.println("Input cam bien rung");
  Serial.println(digitalRead(SW420));

  batTatXe(); // bật tắt xe

  // phatHienTeNga();

  layGocMPU6050();
  Serial.println("co flagMPU6050");
  Serial.println(flagMPU6050);
  delay(1000);

  if (flagTrackingXeMay == 1 )
  {
    // while (1)
    // {
      trackingXeMayRealtime();
      // if (flagTrackingXeMay == 0)
      // {
      //   break;
      // }
  }


    //trackingXeMayRealtime();
    // Serial.println(millisTraking);
    // if ((unsigned long) (millis() -  millisTraking) >= (timeTrankingXeMay))
    // {
    //   loa_2bip();
    //   Serial.println("Stop tracking.............");
    //   flagTrackingXeMay = 0;
    //   flagMPU6050 = 0;
    //   delay(3000);
    // }
  
  if (flagTrackingXeMay == 0)
  {
    Serial.println("\nvao chuc nang chong trom");
    Serial.println(flagRelayOff);
    canhBaoXeBiDatTrom();
  }

  // trackingXeMayRealtime();
  while (SimA7600C.available())
  {
    xuLiTinNhan(SimA7600C.readString());
  }
  while (Serial.available())
  {
    SimA7600C.println(Serial.readString());
  }
}

// cua khoi rf dieu khien
int previousInput = 0; // Biến lưu trạng thái trước đó của INPUT_RF3

void timXe()
{
  int currentInput = digitalRead(RF3);

  if (currentInput != previousInput)
  {
    digitalWrite(loa, 1);
    delay(1000);
    digitalWrite(loa, 0);
  }

  previousInput = currentInput; // Cập nhật trạng thái trước đó cho lần kiểm tra tiếp theo
}

// bat loa
void loa_2bip()
{
  digitalWrite(loa, HIGH);
  delay(300);
  digitalWrite(loa, LOW);
  delay(300);
  digitalWrite(loa, HIGH);
  delay(300);
  digitalWrite(loa, LOW);
  delay(300);
}

// phat hien va cham, rung lac, chong trom
void phatHienVaCham()
{

  if (digitalRead(RF4) == HIGH)
  {
    flagRF4 = 1;
    if (flagRF4_2 == 0)
    {
      loa_2bip();
      flagRF4_2 = 1;
    }
  }

  if (digitalRead(RF4) == LOW)
  {
    flagRF4 = 0;
    if (flagRF4_2 == 1)
    {
      loa_2bip();
      flagRF4_2 = 0;
    }
  }

  if (digitalRead(SW420) == HIGH)
  {
    flagSW420 = 1;
    millisRung = millis();
  }

  if (flagSW420 == 1 && flagRF4 == 1)
  {
    digitalWrite(loa, HIGH);
    Serial.println(millis() - millisRung);
    // lấy giá trị hiện tại trừ đi giá trị trước đó và so sánh với khoản thời gian delay
    if ((unsigned long)(millis() - millisRung) >= (5000))
    {
      flagSW420 = 0;
      digitalWrite(loa, LOW);
    }
  }
}

void canhBaoXeBiDatTrom()
{
  if (flagRelayOff == 1)
  {
    Serial.print("\nxe khi tat may");
    while (serialGPS.available())
    {
      if (gps.encode(serialGPS.read()))
      {
        if (gps.location.isValid())
        {
          Serial.print("\nxe khi tat may");
          if ((gps.location.lat() - previousLatitude) > 0.000015 || (gps.location.lng() - previousLongitude) > 0.000015)
          {
            previousLatitude = gps.location.lat();
            previousLongitude = gps.location.lng();
            for (int i = 0; i < 3; i++)
            {
              // doc so dien thoai tu bo nho eeprom
              phoneNo[i] = docDuLieuTuEEPROM(offsetPhone[i]);
              // neu co do dai = 12 hop le thi
              if (phoneNo[i].length() == 12)
              {
                loa_2bip();
                Serial.println("Canh bao phuong tien bi dat trom");
                // guiTinNhan("Canh bao phuong tien bi dat trom", phoneNo[i]);
                // SimA7600C.println("ATD" + phoneNo[i] + ";");
                // Serial.println("123456789");
                // delay(10000);
                // SimA7600C.println("ATH");

              }
            }
            Serial.print("\ndu lieu true ghjjhffghhhff");
            break;
          }
        }
      }
    }
  }
}

void DoKhoangCach()
{
  if (flagRelayOff == 1)
  {
    Serial.print("\nxe khi tat may");
    float PrevLat = 0.0;
    float PrevLong = 0.0;
    while (serialGPS.available())
    {
      if (gps.encode(serialGPS.read()))
      {
        if (gps.location.isValid())
        {
          float distance = TinyGPSPlus::distanceBetween(PrevLat, PrevLong, gps.location.lat(), gps.location.lng());
          if ((distance >= 100.000000) || (distance <= -100.000000))
          {
            PrevLat = gps.location.lat();
            PrevLong = gps.location.lng();
            for (int i = 0; i < 3; i++)
            {
              // doc so dien thoai tu bo nho eeprom
              phoneNo[i] = docDuLieuTuEEPROM(offsetPhone[i]);
              // neu co do dai = 12 hop le thi
              if (phoneNo[i].length() == 12)
              {
                loa_2bip();
                Serial.println("Canh bao phuong tien bi dat trom");
                // guiTinNhan("Canh bao phuong tien bi dat trom", phoneNo[i]);
                // SimA7600C.println("ATD" + phoneNo[i] + ";");
                // Serial.println("123456789");
                // delay(10000);
                // SimA7600C.println("ATH");
              }
            }
            Serial.print("\ndu lieu true ghjjhffghhhff");
            break;
          }
        }
      }
    }
  }
}

// void canhBaoXeBiDatTrom()
// {
//   if (flagRelayOff == 1)
//   {
//     Serial.print("\nxe khi tat may");
//     while (serialGPS.available())
//     {
//       if (gps.encode(serialGPS.read()))
//       {
//         if (gps.location.isValid())
//         {
//           Serial.print("\nxe khi tat may");
//           float lat = gps.location.lat();
//           float lon = gps.location.lng();
//           float distance = TinyGPSPlus::distanceBetween(lat, lon, 10.832611, 106.660000);
//           if ((gps.location.lat() - previousLatitude) > 0.000015 || (gps.location.lng() - previousLongitude) > 0.000015)
//           {
//             previousLatitude = gps.location.lat();
//             previousLongitude = gps.location.lng();
//             for (int i = 0; i < 3; i++)
//             {
//               // doc so dien thoai tu bo nho eeprom
//               phoneNo[i] = docDuLieuTuEEPROM(offsetPhone[i]);
//               // neu co do dai = 12 hop le thi
//               if (phoneNo[i].length() == 12)
//               {
//                 loa_2bip();
//                 Serial.println("Canh bao phuong tien bi dat trom");
//                 // guiTinNhan("Canh bao phuong tien bi dat trom", phoneNo[i]);
//                 // SimA7600C.println("ATD" + phoneNo[i] + ";");
//                 // Serial.println("123456789");
//                 // delay(10000);
//                 // SimA7600C.println("ATH");

//               }
//             }
//             Serial.print("\ndu lieu true ghjjhffghhhff");
//             break;
//           }
//         }
//       }
//     }
//   }
// }
void batTatXe()
{
  int dem = 0;
  if (digitalRead(RF1) == HIGH)
  {
    Serial.println("789");
    for (dem = 0; dem <= 10000; dem++)
    {
      Serial.println("123");
      phatHienVaCham();
      int currentInput = digitalRead(RF3);
      if (currentInput != previousInput)
      {
        Serial.println("\n456");
        digitalWrite(SW420, LOW);
        digitalWrite(loa, LOW);
        digitalWrite(relay, HIGH);
        flagSW420 = 0;
        // flagRelayOn = 1;
        flagRelayOff = 0;
        Serial.println("\n123456789");
        Serial.println("flag rely on");
        Serial.println(flagRelayOn);
        delay(1000);
        break;
      }
    }
    if (dem == 10001)
    {
      flagRelayOn = 0;
      flagRelayOff = 1;
      digitalWrite(loa, 1);
      delay(10000);
      digitalWrite(loa, 0);
    }
  }
  else if (flagRelayOn == 1)
  {
    digitalWrite(relay, HIGH);
    Serial.println("flag rely on...............");
    Serial.println(flagRelayOn);
    flagRelayOff = 0;
  }

  if (digitalRead(RF2) == HIGH || flagRelayOff == 1)
  {
    digitalWrite(relay, LOW);
    flagRelayOn = 0;
    flagRelayOff = 1;
  }
}

void phatHienTeNga()
{
  // neu goc nghieng lon hon 50 theo truc x va y
  if (abs(gocX - gocXBanDau) > gocTeNga || abs(gocY - gocYBanDau) > gocTeNga)
  {
    if ((unsigned long)(millis() - millisTaiNan) >= 30000)
    {
      Serial.println("thoi gian te nga");
      Serial.println(millis() - millisTaiNan);
      millisTaiNan = 0;
      for (int i = 0; i < 3; i++)
      {
        // doc so dien thoai tu bo nho eeprom
        phoneNo[i] = docDuLieuTuEEPROM(offsetPhone[i]);
        // neu co do dai = 12 hop le thi
        if (phoneNo[i].length() == 12)
        {
          // gui canh bao phuong tien te nga den so dien thoai
          guiTinNhan("Canh bao phuong tien te nga", phoneNo[i]);
          guiViTri(phoneNo[i]);
          SimA7600C.println("ATD" + phoneNo[i] + ";");
          delay(10000);
          // ATH kết thúc
          SimA7600C.println("ATH");
        }
      }
      while (1)
      {
        layGocMPU6050();
        Serial.println("vong lap while");
        Serial.println("co flagMPU6050");
        Serial.println(flagMPU6050);
        Serial.println("\n lay goc: x");
        Serial.println(gocX);
        Serial.println("\n lay goc: y");
        Serial.println(gocY);
        delay(1000);
        if (abs(gocX - gocXBanDau) > gocTeNga || abs(gocY - gocYBanDau) > gocTeNga)
        {
          for (int i = 0; i < 3; i++)
          {
            // doc so dien thoai tu bo nho eeprom
            phoneNo[i] = docDuLieuTuEEPROM(offsetPhone[i]);
            // neu co do dai = 12 hop le thi
            if (phoneNo[i].length() == 12)
            {
              SimA7600C.println("ATD" + phoneNo[i] + ";");
              delay(10000);
              SimA7600C.println("ATH");
            }
          }
        }
        else if (abs(gocX - gocXBanDau) < gocTeNga || abs(gocY - gocYBanDau) < gocTeNga)
        {
          Serial.println("thoat khoi vong lap while");
          break;
        }
      }
    }
  }
}

void extract_sms(String buff)
{
  unsigned int index; // Khai báo biến index kiểu unsigned int để lưu vị trí (index) của một ký tự trong chuỗi.

  index = buff.indexOf(","); // Tìm vị trí đầu tiên của dấu phẩy (,) trong chuỗi buff và gán nó cho biến index.

  // Trích xuất một phần của chuỗi buff bắt đầu từ vị trí 1 đến vị trí index - 1 và lưu kết quả vào biến smsStatus.
  smsStatus = buff.substring(1, index - 1);

  // Xóa các ký tự từ vị trí 0 đến index + 2, loại bỏ thông tin đã được xử lý khỏi chuỗi buff.
  buff.remove(0, index + 2);

  // Trích xuất 12 ký tự đầu tiên của chuỗi buff và lưu vào biến senderNumber.
  senderNumber = buff.substring(0, 12);

  // Tìm vị trí của chuỗi "\r" trong buff và loại bỏ nó cùng với bất kỳ khoảng trắng nào ở đầu và cuối chuỗi buff.
  buff.remove(0, buff.indexOf("\r"));
  buff.trim();

  // Tìm vị trí xuất hiện đầu tiên của chuỗi "\n\r" trong buff.
  index = buff.indexOf("\n\r");

  // Trích xuất một phần của buff từ vị trí 0 đến index và loại bỏ bất kỳ khoảng trắng nào ở đầu và cuối chuỗi.
  buff = buff.substring(0, index);
  buff.trim();

  // Lưu nội dung của tin nhắn SMS đã được xử lý vào biến msg và sau đó đặt msg thành chữ thường (lowercase).
  msg = buff;
  buff = "";
  msg.toLowerCase();

  // Trích xuất 3 ký tự đầu tiên của msg và lưu vào biến tempcmd.
  String tempcmd = msg.substring(0, 3);

  // In ra màn hình cổng nối tiêu chuẩn (Serial Monitor) để kiểm tra giá trị của tempcmd.
  Serial.println(tempcmd);

  // Kiểm tra xem tempcmd có giống với "r1=", "r2=", hoặc "r3=" không.
  if (tempcmd.equals("r1=") || tempcmd.equals("r2=") ||
      tempcmd.equals("r3=") || tempcmd.equals("r4=") ||
      tempcmd.equals("r5="))
  {

    // Trích xuất phần cuối cùng của msg (một số điện thoại) và lưu vào biến tempPhone.
    tempPhone = msg.substring(3, 16);
    Serial.println("tempPhone:ZZ");
    Serial.println(tempPhone);

    // Đặt giá trị của msg thành tempcmd.
    msg = tempcmd;
  }
}

// so sanh có số đt nào giống trong eeprom ko
boolean comparePhone(String number)
{
  // Serial.println("so sanh so dien thoai");
  Serial.println(tongSoDienThoai); // In ra tổng số điện thoại đã lưu trữ
  boolean flag = 0;                // Khởi tạo một biến boolean `flag` và đặt giá trị ban đầu là 0 (false).
  //--------------------------------------------------
  // Duyệt qua danh sách số điện thoại đã lưu trữ để so sánh với số điện thoại đầu vào.
  for (int i = 0; i < tongSoDienThoai; i++)
  {
    // Đọc số điện thoại từ bộ nhớ EEPROM bằng cách gọi hàm `docDuLieuTuEEPROM`.
    phoneNo[i] = docDuLieuTuEEPROM(offsetPhone[i]);
    // So sánh số điện thoại đọc được từ EEPROM với số điện thoại đầu vào.
    if (phoneNo[i].equals(number))
    {
      Serial.println(phoneNo[i]); // In ra số điện thoại khớp.
      flag = 1;                   // Đặt biến `flag` thành 1 (true) để chỉ ra rằng có một kết quả khớp.
      break;                      // Thoát khỏi vòng lặp vì đã tìm thấy một kết quả khớp.
    }
  }
  //--------------------------------------------------
  return flag;
}

// addrOffset: Một số nguyên, đại diện cho địa chỉ bắt đầu của bộ nhớ EEPROM mà bạn muốn ghi dữ liệu vào.
// strToWrite: Tham chiếu hằng đến một chuỗi (String) chứa dữ liệu mà bạn muốn ghi vào EEPROM.
// viet text vao eeprom
void vietDuLieuVaoEEPROM(int addrOffset, const String &strToWrite)
{
  // Serial.println("gi vao bo nho eeprom");
  byte len = 12;
  // EEPROM.write(addrOffset, len);
  for (int i = 0; i < len; i++)
  {
    // được sử dụng để ghi ký tự tại vị trí i trong chuỗi strToWrite vào bộ nhớ EEPROM tại địa chỉ addrOffset + i
    EEPROM.write(addrOffset + i, strToWrite[i]);
    // Serial.println("gi vao bo nho eeprom 2 ");
  }
  EEPROM.commit(); // lưu trữ thay đổi vào EEPROM
}

String docDuLieuTuEEPROM(int addrOffset)
{
  int len = 12;
  char data[len + 1];
  for (int i = 0; i < len; i++)
  {
    data[i] = EEPROM.read(addrOffset + i);
  }
  data[len] = '\0';
  return String(data);
}

void xuLiTinNhan(String buff)
{
  // Serial.println("buff dau tien");
  Serial.println(buff); // In ra chuỗi `buff` trên cổng Serial (để mục đích gỡ lỗi).

  unsigned int len, index; // Khai báo hai biến kiểu unsigned int: len và index.
  // Remove sent "AT Command" from the response string.
  index = buff.indexOf("\r"); // Tìm vị trí đầu tiên của dấu xuống dòng '\r' trong chuỗi `buff`.
  buff.remove(0, index + 2);  // Loại bỏ phần "AT Command" và ký tự xuống dòng từ `buff`.

  // Serial.println("sau khi loai bo AT");
  // Serial.println(buff);
  buff.trim(); // Loại bỏ các khoảng trắng ở đầu và cuối chuỗi.
  // Serial.println("sau khi loai bo khoang trang");
  // Serial.println(buff);

  // Kiểm tra nếu chuỗi `buff` không phải là "OK".
  if (buff != "OK")
  {
    index = buff.indexOf(":");             // Tìm vị trí đầu tiên của dấu hai chấm ':' trong chuỗi `buff`.
    String cmd = buff.substring(0, index); // Trích xuất phần từ đầu đến dấu hai chấm ':' và lưu vào biến `cmd`.
    // Serial.println("bien cmd");
    // Serial.println(cmd);
    cmd.trim(); // Loại bỏ các khoảng trắng ở đầu và cuối chuỗi `cmd`.
    // Serial.println("Loại bỏ các khoảng trắng ở đầu và cuối chuỗi `cmd");
    // Serial.println(cmd);
    buff.remove(0, index + 2); // Loại bỏ phần đã trích xuất và ký tự hai chấm ':' từ `buff`.
    // Serial.println(" Loại bỏ phần đã trích xuất và ký tự hai chấm ':' từ `buff`");
    // Serial.println(buff);

    // Serial.println("--------------------");
    // Serial.println(cmd);
    // Serial.println("--------------------");

    if (cmd == "+CMTI")
    {
      flagNgatMpu6065 = 1;
      // get newly arrived memory location and store it in temp
      //  Lấy vị trí bộ nhớ mới nhận và lưu vào biến `temp`.
      index = buff.indexOf(",");
      IndexSms = index; // Gán giá trị index cho biến IndexSms.
      // Serial.println("gia tri bien index");
      // Serial.println(index);
      String temp = buff.substring(index + 1, buff.length()); // Trích xuất phần sau dấu phẩy ',' và lưu vào `temp`.
      // Serial.println("Trích xuất phần sau dấu phẩy ',' và lưu vào `temp`.");
      temp = "AT+CMGR=" + temp + "\r"; // Tạo lệnh AT để lấy tin nhắn tại vị trí bộ nhớ `temp`.
      // get the message stored at memory location "temp"
      SimA7600C.println(temp); // Gửi lệnh AT đến SimA7600C.
      Serial.println("....."); // In ra dấu chấm để gỡ lỗi.
    }

    else if (cmd == "+CMGR")
    {
      // Serial.println("vao hamf else if");
      extract_sms(buff); // Gọi hàm `extract_sms` để trích xuất thông tin tin nhắn từ `buff`.
      // Serial.println("testttttttttttttttttt");
      // Gửi lệnh AT để xóa tin nhắn tại vị trí `IndexSms`.
      SimA7600C.println("AT+CMGD=" + IndexSms);
      if (msg.equals("r") && flagDangKy == 0)
      {
        // && phoneNo[0].length() == 12
        vietDuLieuVaoEEPROM(offsetPhone[0], senderNumber); // Lưu số điện thoại vào EEPROM.
        phoneNo[0] = senderNumber;                         // Gán số điện thoại vào mảng `phoneNo`.
        String text = "Number is Registered: ";
        text = text + senderNumber; // Gửi phản hồi về số điện thoại đã đăng ký.
        guiTinNhan("Number is Registered", senderNumber);
        flagDangKy = 1;
      }

      if (comparePhone(senderNumber))
      {
        Serial.println("vao do action");
        do_action(senderNumber);
        flagNgatMpu6065 = 0;
      }
      else
      {
        guiTinNhan("so dien thoai chua duoc dang ki", senderNumber);
      }
    }
  }
  else
  {
    // The result of AT Command is "OK"
  }
}

void xuLiTinNhanRelay(String buff)
{
  Serial.println(buff);    // In ra chuỗi `buff` trên cổng Serial (để mục đích gỡ lỗi).
  unsigned int len, index; // Khai báo hai biến kiểu unsigned int: len và index.
  // Remove sent "AT Command" from the response string.
  index = buff.indexOf("\r"); // Tìm vị trí đầu tiên của dấu xuống dòng '\r' trong chuỗi `buff`.
  buff.remove(0, index + 2);  // Loại bỏ phần "AT Command" và ký tự xuống dòng từ `buff`.
  buff.trim();                // Loại bỏ các khoảng trắng ở đầu và cuối chuỗi.
  if (buff != "OK")
  {
    index = buff.indexOf(":");             // Tìm vị trí đầu tiên của dấu hai chấm ':' trong chuỗi `buff`.
    String cmd = buff.substring(0, index); // Trích xuất phần từ đầu đến dấu hai chấm ':' và lưu vào biến `cmd`.
    cmd.trim();                            // Loại bỏ các khoảng trắng ở đầu và cuối chuỗi `cmd`.
    buff.remove(0, index + 2);             // Loại bỏ phần đã trích xuất và ký tự hai chấm ':' từ `buff`.
    if (cmd == "+CMTI")
    {
      flagNgatMpu6065 = 1;
      index = buff.indexOf(",");
      IndexSms = index;                                       // Gán giá trị index cho biến IndexSms.
      String temp = buff.substring(index + 1, buff.length()); // Trích xuất phần sau dấu phẩy ',' và lưu vào `temp`.
      temp = "AT+CMGR=" + temp + "\r";                        // Tạo lệnh AT để lấy tin nhắn tại vị trí bộ nhớ `temp`.
      SimA7600C.println(temp);                                // Gửi lệnh AT đến SimA7600C.
      Serial.println(".....");                                // In ra dấu chấm để gỡ lỗi.
    }
    else if (cmd == "+CMGR")
    {
      extract_sms(buff); // Gọi hàm `extract_sms` để trích xuất thông tin tin nhắn từ `buff`.
      SimA7600C.println("AT+CMGD=" + IndexSms);
      if (msg.equals("r") && flagDangKy == 0)
      {
        // && phoneNo[0].length() == 12
        vietDuLieuVaoEEPROM(offsetPhone[0], senderNumber); // Lưu số điện thoại vào EEPROM.
        phoneNo[0] = senderNumber;                         // Gán số điện thoại vào mảng `phoneNo`.
        String text = "Number is Registered: ";
        text = text + senderNumber; // Gửi phản hồi về số điện thoại đã đăng ký.
        guiTinNhan("Number is Registered", senderNumber);
        flagDangKy = 1;
      }
      if (comparePhone(senderNumber))
      {
        Serial.println("vao do action_y_n");
        do_action_y_n(senderNumber);
        flagNgatMpu6065 = 0;
      }
    }
  }
  else
  {
    // The result of AT Command is "OK"
  }
}

// gửi tin nhăn đến điện thoại
void guiTinNhan(String text, String Phone)
{
  // đặt chế độ SMS văn bản và đợi module phản hồi bằng "OK".
  SimA7600C.print("AT+CMGF=1\r");
  delay(1000);
  // chuyển sang chế độ ghi nội dung tin nhắn SMS.
  SimA7600C.print("AT+CMGS=\"" + Phone + "\"\r");
  delay(1000);
  SimA7600C.print(text);
  delay(100);
  // kết thúc và gửi tin nhắn
  SimA7600C.write(0x1A); // ascii code for ctrl-26 //sim800.println((char)26); //ascii code for ctrl-26
  delay(1000);
  Serial.println("SMS Sent Successfully.");

  smsStatus = "";
  senderNumber = "";
  msg = "";
  tempPhone = "";
}

// gửi vị trí đến điện thoại
void guiViTri(String phoneNumber)
{
  Serial.println("Vao day roi!! check gps");
  boolean newData = false;
  // Lặp trong vòng 5 giây hoặc cho đến khi có dữ liệu từ GPS module.
  for (unsigned long Time_gps = millis(); millis() - Time_gps < 5000;)
  {
    while (serialGPS.available() > 0) // Kiểm tra xem có dữ liệu từ cổng nối tiếp của GPS module hay không.
    {
      if (gps.encode(serialGPS.read())) // Giải mã dữ liệu từ GPS module.
      {
        if (gps.location.isValid())
        {                 // Kiểm tra xem dữ liệu vị trí từ GPS có hợp lệ hay không.
          newData = true; // Đặt biến `newData` thành true để chỉ ra rằng có dữ liệu vị trí mới.
          break;          // Thoát khỏi vòng lặp vì đã có dữ liệu vị trí mới.
        }
      }
    }
  }
  if (newData)
  {
    newData = false;
    // Lấy giá trị vĩ độ (latitude) và kinh độ (longitude) từ GPS và chuyển chúng thành chuỗi.
    String latitude = String(gps.location.lat(), 6);
    String longitude = String(gps.location.lng(), 6);

    // Tạo một chuỗi văn bản chứa thông tin vị trí và liên kết Google Maps.
    String text = "Latitude= " + latitude + "\n" + "\r" + "      ";
    text += "Longitude= " + longitude + "\n" + +"\r" + "       ";
    text += "http://maps.google.com/maps?q=" + latitude + "," + longitude;
    guiTinNhan(text, phoneNumber);
    delay(3000);
  }
  //-----------------------------------------------------------------
}

void dayDuLieuLenFirebase(String data)
{
  // Start HTTP connection
  SimA7600C.println("AT+HTTPINIT");
  delay(500);
  // Set the HTTP URL - Firebase URL and FIREBASE SECRET
  SimA7600C.println("AT+HTTPPARA=\"URL\"," + firebaseUrl + ".json?auth=" + firebaseScret);
  // delay(500);
  // Setting up content type
  SimA7600C.println("AT+HTTPPARA=\"POST\",\"application/json\"");
  // waitResponse();
  delay(500);
  // Setting up Data Size
  //+HTTPACTION: 1,601,0 - error occurs if data length is not correct
  SimA7600C.println("AT+HTTPDATA=" + String(data.length()) + ",10000");
  delay(500);
  // Sending Data
  SimA7600C.println(data);
  // waitResponse();
  delay(500);
  // Sending HTTP POST request
  SimA7600C.println("AT+HTTPACTION=1");
  for (uint32_t start = millis(); millis() - start < 3000;)
  {
    while (!SimA7600C.available());
    String response = SimA7600C.readString();
    if (response.indexOf("+HTTPACTION:") > 0)
    {
      Serial.println(response);
      break;
    }
  }
  // Stop HTTP connection
  SimA7600C.println("AT+HTTPTERM");
  delay(500);
}

void trackingXeMayRealtime()
{
  boolean newData = false;
  while (serialGPS.available()) {
    if (gps.encode(serialGPS.read())) {
      if (gps.location.isValid())
      {
        newData = true;
        break;
      }
    }
  }
  if (newData) {
    newData = false;
    String latitude, longitude;
    latitude = String(gps.location.lat(), 6); // Latitude in degrees (double)
    longitude = String(gps.location.lng(), 6); // Longitude in degrees (double)

    Serial.print("Latitude= ");
    Serial.print(latitude);
    Serial.print(" Longitude= ");
    Serial.println(longitude);

    String gpsData = "{";
    gpsData += "\"latitude\":" + latitude + ",";
    gpsData += "\"longitude\":" + longitude + "";
    gpsData += "}";
    dayDuLieuLenFirebase(gpsData);
    Serial.println(millisTraking);
    if ((unsigned long) (millis() -  millisTraking) >= 10000)
    {
      Serial.println("Done waiting. Going to next tracking");
    }

  }
}

// void trackingXeMayRealtime()
// {
//   boolean newData = false;
//   while (serialGPS.available())
//   {
//     // Serial.print("\ndu lieu true 1");
//     // Serial.print(newData);
//     if (gps.encode(serialGPS.read()))
//     {
//       // Serial.print("\ndu lieu true 2");
//       // Serial.print(newData);
//       if (gps.location.isValid())
//       {
//         // Serial.print("\ndu lieu true 3 ");
//         // Serial.print(newData);
//         // Serial.print("123456789\n");
//         Serial.print("\nsai so kinh do");
//         Serial.print(gps.location.lat() - previousLatitude);
//         Serial.print("\nsai so kinh vi do");
//         Serial.print(gps.location.lng() - previousLongitude);
//         // delay(1000);
//         if ((gps.location.lat() - previousLatitude) > 0.000015 || (gps.location.lng() - previousLongitude) > 0.000015)
//         {
//           newData = true;
//           previousLatitude = gps.location.lat();
//           previousLongitude = gps.location.lng();
//           Serial.print("\ndu lieu true ghjjhffghhhff");
//           Serial.print(newData);
//           break;
//         }
//       }
//     }
//   }
//   if (newData)
//   {
//     Serial.print("gui du lieu");
//     Serial.print(newData);
//     newData = false;
//     Serial.print("\n");
//     Serial.print(newData);
//     String latitude, longitude;
//     latitude = String(gps.location.lat(), 6);  // Latitude in degrees (double)
//     longitude = String(gps.location.lng(), 6); // Longitude in degrees (double)

//     Serial.print("Latitude= ");
//     Serial.print(latitude);
//     Serial.print(" Longitude= ");
//     Serial.println(longitude);

//     String gpsData = "{";
//     gpsData += "\"latitude\":" + latitude + ",";
//     gpsData += "\"longitude\":" + longitude + "";
//     gpsData += "}";
//     dayDuLieuLenFirebase(gpsData);
//   }
// }

void do_action_y_n(String phoneNumber)
{
  Serial.println(msg);
  if (msg == "y relay on")
  {
    flagRelayOn = 1;
    flagRelayOff = 0;
    Serial.println("co relay on");
    Serial.println(flagRelayOn);
    delay(1000);
    guiTinNhan("relay on!", phoneNumber);
    thoatVongLapRelay = true;
  }
  else if (msg == "n relay on")
  {
    guiTinNhan("da huy bat relay", phoneNumber);
    thoatVongLapRelay = true;
  }

  else if (msg == "y relay off")
  {
    flagRelayOn = 0;
    flagRelayOff = 1;
    Serial.println("co relay off");
    Serial.println(flagRelayOn);
    delay(1000);
    guiTinNhan("relay off!", phoneNumber);
    thoatVongLapRelay = true;
  }
  else if (msg == "n relay off")
  {
    guiTinNhan("da huy tat relay", phoneNumber);
    thoatVongLapRelay = true;
  }

  smsStatus = "";
  senderNumber = "";
  msg = "";
  tempPhone = "";
}

void do_action(String phoneNumber)
{
  Serial.println(msg);
  if (msg == "send location")
  {
    flagMPU6050 = 1;
    guiViTri(phoneNumber);
    flagMPU6050 = 0;
  }

  else if (msg == "r2=")
  {
    Serial.println(offsetPhone[1]);
    vietDuLieuVaoEEPROM(offsetPhone[1], tempPhone);
    phoneNo[1] = tempPhone;
    String text = "Phone2 is Registered: ";
    text = text + tempPhone;
    guiTinNhan(text, phoneNumber);
  }

  else if (msg == "r3=")
  {
    vietDuLieuVaoEEPROM(offsetPhone[2], tempPhone);
    phoneNo[2] = tempPhone;
    String text = "Phone3 is Registered: ";
    text = text + tempPhone;
    guiTinNhan(text, phoneNumber);
  }

  else if (msg == "r4=")
  {
    vietDuLieuVaoEEPROM(offsetPhone[3], tempPhone);
    phoneNo[3] = tempPhone;
    String text = "Phone4 is Registered: ";
    text = text + tempPhone;
    guiTinNhan(text, phoneNumber);
  }
  else if (msg == "r5=")
  {
    vietDuLieuVaoEEPROM(offsetPhone[4], tempPhone);
    phoneNo[4] = tempPhone;
    String text = "Phone5 is Registered: ";
    text = text + tempPhone;
    guiTinNhan(text, phoneNumber);
  }
  else if (msg == "list")
  {

    String text = "";
    if (phoneNo[0])
      text = text + phoneNo[0] + "\r\n             ";
    if (phoneNo[1])
      text = text + phoneNo[1] + "\r\n             ";
    if (phoneNo[2])
      text = text + phoneNo[2] + "\r\n             ";
    if (phoneNo[3])
      text = text + phoneNo[3] + "  \r\n             ";
    if (phoneNo[4])
      text = text + phoneNo[4] + "  \r\n             ";
    guiTinNhan(text, phoneNumber);
  }

  else if (msg == "del=1")
  {
    vietDuLieuVaoEEPROM(offsetPhone[0], "");
    phoneNo[0] = "";
    guiTinNhan("Phone1 is deleted.", phoneNumber);
  }

  else if (msg == "del=2")
  {
    vietDuLieuVaoEEPROM(offsetPhone[1], "");
    phoneNo[1] = "";
    guiTinNhan("Phone2 is deleted.", phoneNumber);
  }

  else if (msg == "del=3")
  {
    vietDuLieuVaoEEPROM(offsetPhone[2], "");
    phoneNo[2] = "";
    guiTinNhan("Phone3 is deleted.", phoneNumber);
  }

  else if (msg == "del=4")
  {
    vietDuLieuVaoEEPROM(offsetPhone[3], "");
    phoneNo[3] = "";
    guiTinNhan("Phone4 is deleted.", phoneNumber);
  }

  else if (msg == "del=5")
  {
    vietDuLieuVaoEEPROM(offsetPhone[4], "");
    phoneNo[4] = "";
    guiTinNhan("Phone5 is deleted.", phoneNumber);
  }

  else if (msg == "relay on")
  {
    guiTinNhan("y relay on de bat, hoac gui n relay on, hoac sau 5 phut se tu huy qua trinh relay on", phoneNumber);
    timerWhileOnOff = millis();
    while (1)
    {
      Serial.print("xin chao");
      while (SimA7600C.available())
      {
        xuLiTinNhanRelay(SimA7600C.readString());
      }
      while (Serial.available())
      {
        SimA7600C.println(Serial.readString());
      }
      // Kiểm tra nếu đã đủ 5 phút, thoát khỏi vòng lặp
      if (millis() - timerWhileOnOff >= 300000)
      {
        break;
      }
      if (thoatVongLapRelay)
      {
        thoatVongLapRelay = false;
        break;
      }
    }
  }

  else if (msg == "relay off")
  {
    guiTinNhan("y relay off de tat, hoac gui n relay off, hoac sau 5 phut se tu huy qua trinh relay off", phoneNumber);
    timerWhileOnOff = millis();
    while (1)
    {
      while (SimA7600C.available())
      {
        xuLiTinNhanRelay(SimA7600C.readString());
      }
      while (Serial.available())
      {
        SimA7600C.println(Serial.readString());
      }
      // Kiểm tra nếu đã đủ 5 phút, thoát khỏi vòng lặp
      if (millis() - timerWhileOnOff >= 300000)
      {
        break;
      }
      if (thoatVongLapRelay)
      {
        thoatVongLapRelay = false;
        break;
      }
    }
  }
  // else if (msg == "tracking vehicle 1p")
  // {
  //   flagTrackingXeMay = 1;
  //   millisTraking = millis();
  //   timeTrankingXeMay = 60000;
  //   flagMPU6050 = 1;
  //   guiTinNhan("tracking real time 1p enable!", phoneNumber);
  // }

  // else if (msg == "tracking vehicle 5p")
  // {
  //   flagTrackingXeMay = 1;
  //   millisTraking = millis();
  //   timeTrankingXeMay = 240000;
  //   flagMPU6050 = 1;
  //   guiTinNhan("tracking real time 5p enable!", phoneNumber);
  // }
  else if (msg == "tracking vehicle")
  {
    flagTrackingXeMay = 1;
    millisTraking = millis();
    flagMPU6050 = 1;
    guiTinNhan("tracking real time enable!", phoneNumber);
  }

  else if (msg == "stop tracking vehicle")
  {
    flagTrackingXeMay = 0;
    flagMPU6050 = 0;
    guiTinNhan("stop tracking real times", phoneNumber);
  }

  else if (msg == "del=all")
  {
    vietDuLieuVaoEEPROM(offsetPhone[0], "");
    vietDuLieuVaoEEPROM(offsetPhone[1], "");
    vietDuLieuVaoEEPROM(offsetPhone[2], "");
    vietDuLieuVaoEEPROM(offsetPhone[3], "");
    vietDuLieuVaoEEPROM(offsetPhone[4], "");

    phoneNo[0] = "";
    phoneNo[1] = "";
    phoneNo[2] = "";
    phoneNo[3] = "";
    phoneNo[4] = "";

    offsetPhone[0] = NULL;
    offsetPhone[1] = NULL;
    offsetPhone[2] = NULL;
    offsetPhone[3] = NULL;
    offsetPhone[4] = NULL;

    flagDangKy = 0;
    guiTinNhan("All phone numbers are deleted.", phoneNumber);
  }
  else
  {
    guiTinNhan("cu phap sai", phoneNumber);
  }

  smsStatus = "";
  senderNumber = "";
  msg = "";
  tempPhone = "";
} 

// ham lay goc
void layGocMPU6050()
{
  mpu6050.update();
  gocX = mpu6050.getAngleX();
  gocY = mpu6050.getAngleY();
  gocZ = mpu6050.getAngleZ();
  Serial.print("angleX : ");
  Serial.print(gocX);
  Serial.print("\tangleY : ");
  Serial.print(gocY);
  Serial.print("\tangleZ : ");
  Serial.println(gocZ);
  Serial.println("=======================================================\n");
  // timer = millis();
}

void testGPS()
{
  delay(1000);
  Serial.print("vao ham GPS");
  while (serialGPS.available())
  {
    if (gps.encode(serialGPS.read()))
    {
      if (gps.location.isValid())
      {
        // Đã nhận được dữ liệu GPS hợp lệ
        // Trích xuất thông tin vĩ độ và kinh độ
        float latitude = gps.location.lat();
        float longitude = gps.location.lng();
        Serial.print("Latitude: ");
        Serial.println(latitude, 6); // In vĩ độ với 6 chữ số thập phân
        Serial.print("Longitude: ");
        Serial.println(longitude, 6); // In kinh độ với 6 chữ số thập phân

        // Bạn có thể sử dụng các giá trị latitude và longitude ở đây cho mục ích của bạn
      }
    }
  }
}
