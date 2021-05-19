# ESP32の使用で得た知見
ESP32は(たぶん)COREで初めて使うマイコンボードです．使い方が分からないことも多いと思うので，使用していく中で得られた知見をここにまとめていきましょう．<br>

・[準備](https://github.com/CORE-since2020/Duo/blob/main/ESP32%E3%81%A7%E5%BE%97%E3%81%9F%E7%9F%A5%E8%A6%8B.md#%E6%BA%96%E5%82%99)<br>
・[プログラムの書き込み](https://github.com/CORE-since2020/Duo/blob/main/ESP32%E3%81%A7%E5%BE%97%E3%81%9F%E7%9F%A5%E8%A6%8B.md#%E3%83%97%E3%83%AD%E3%82%B0%E3%83%A9%E3%83%A0%E6%9B%B8%E3%81%8D%E8%BE%BC%E3%81%BF)<br>
・[シリアル通信](https://github.com/CORE-since2020/Duo/blob/main/ESP32%E3%81%A7%E5%BE%97%E3%81%9F%E7%9F%A5%E8%A6%8B.md#%E3%82%B7%E3%83%AA%E3%82%A2%E3%83%AB%E9%80%9A%E4%BF%A1)<br>
・[I2C通信](https://github.com/CORE-since2020/Duo/blob/main/ESP32%E3%81%A7%E5%BE%97%E3%81%9F%E7%9F%A5%E8%A6%8B.md#i2c%E9%80%9A%E4%BF%A1)<br>
・[アナログ入力](https://github.com/CORE-since2020/Duo/blob/main/ESP32%E3%81%A7%E5%BE%97%E3%81%9F%E7%9F%A5%E8%A6%8B.md#%E3%82%A2%E3%83%8A%E3%83%AD%E3%82%B0%E5%85%A5%E5%8A%9B)<br>
・[タイマー割込み](https://github.com/CORE-since2020/Duo/blob/main/ESP32%E3%81%A7%E5%BE%97%E3%81%9F%E7%9F%A5%E8%A6%8B.md#%E3%82%BF%E3%82%A4%E3%83%9E%E3%83%BC%E5%89%B2%E8%BE%BC%E3%81%BF)<br>
・[SD](https://github.com/CORE-since2020/Duo/blob/main/ESP32%E3%81%A7%E5%BE%97%E3%81%9F%E7%9F%A5%E8%A6%8B.md#sd)<br>
・[サーボモーター](https://github.com/CORE-since2020/Duo/blob/main/ESP32%E3%81%A7%E5%BE%97%E3%81%9F%E7%9F%A5%E8%A6%8B.md#%E3%82%B5%E3%83%BC%E3%83%9C%E3%83%A2%E3%83%BC%E3%82%BF%E3%83%BC)

## ESP32のピン配置
![ESP32ピン配置](https://github.com/CORE-since2020/Duo/blob/main/image/esp32_devkitc_pinout_01.png?raw=true)
## 準備
1. PCと接続
	- ESP32の接続ポートはUSBのmicro typeB
2. (Windows)デバイスマネージャーを開いてESP32が認識されているか確認する
	
	- もしドライバーがインストールされていなかったら自動更新でインストールする
3. Arduino IDEにESP32のボード情報をインストールする
	1. Arduino IDEを開く
	
	2. "ファイル"→"環境設定"を開く
	
	3. 追加のボードマネージャのURLの部分に次のURLをコピーして貼り付ける<br>					https://dl.espressif.com/dl/package_esp32_index.json
	4. OKボタンを押す
	5. "ツール"→"ボード"→"ボードマネージャ"を開く
	6. 検索欄に"esp32"と入力して検索
	7. インストールを押す
	8. インストールが完了したら"ツール"→"ボード"を開いて一覧に"ESP32 Dev Module"があることを確認して選択する
4. COMポートを設定する(PC-ESP32のシリアル通信の設定)
	1．(Windows)デバイスマネージャーに出てくる"Silicon Labs CP210x USB to UART Bridge"に表示されているCOM番号を控える
	2. "ツール"→"シリアルポート"を開き，先ほど控えたCOM番号を選択する

### 注意：Macを使っている方やPythonの実行環境を仮想環境上で行っている方へ
1. `/Users/xxxxx/Library/Arduino15/packages/esp32/hardware/esp32/1.x.x/platform.txt`を開き，7行目にある`tools.esptool_py.cmd=esptool`となっている部分を`tools.esptool_py.cmd=esptool.py`と書き換えて保存する
2. `/Users/xxxxx/Library/Arduino15/packages/esp32/hardware/esp32/1.x.x/tools/esptool.py`を`/Users/xxxxx/Library/Arduino15/packages/esp32/tools/esptool_py/2.x.x/`にコピーする。
3. コピーした`esptool.py`に実行権限を与える。（ターミナルなどで`chmod +x <ファイルのpath>`を実行すれば良い。）
4. ここからはPythonをいじってる方に生じるエラーで`ImportError: No module named serial`となどと出るはず。仮想環境上でやるのであれば，仮想環境上で
```
pip3 install serial
pip3 install pyserial
```
を実行する。

5. インストールが完了したら`which python3`で出力されるpython3のpathをesptool.pyの1行目に書き換えてあげればよい。
6. Arduino IDEを再起動


## プログラム書き込み
1. まずArduino IDEの左上のレ点を押してコンパイルできるか確かめる(デバック)
2. この時ファイルを保存していないと自動的に保存するように促されるので，任意のディレクトリに保存
3. デバック完了したらレ点の隣の矢印を押す
4. ウィンドウ下に"書き込み完了しました"と表示されればOK
5. 通常のArduinoならここから自動でプログラムが開始するが，ESP32はどうやらそうじゃないらしい．ENと書いてるタクトスイッチ(リセットボタン)を押すとプログラムが開始する．書き込んでも動かないからといって故障と勘違いしないように．
## シリアル通信
- 初期のシリアル通信でESP32側からなんか文字列送ってくる
	- 多分ボードの情報が表示される
	- ESP32の起動時のボーレートがデフォルトで115200bpsのようで，これ以外のボーレートで通信開始すると文字化けする
	- もちろんsetup()でSerial.begin(baudrate)でボーレート設定すれば，最初を除いて任意のボーレートで通信できる
	- とりあえずこの初期の読み出しはそこまで重要じゃないのでたとえ文字化けしてても無視(任意のボーレートで良さそう)
- UARTのポートはデフォルト2組使える

	|変数名|RXピン|TXピン|
	|:---:|:---:|:---:|
	|Serial|3|1|
	|Serial2|16|17|

- ~~Serial1も存在するがデフォルトでは使用できない．今回はGPSモジュールと無線機の2組のみでUARTを使用するため割愛する~~
- Serial1はRxをGPIO26，TxをGPIO27とすると使用可能であることが分かった．プログラムの書き方はSerial2と同様．
- プログラミングを書き込む際，PCとESP32のシリアル通信はSerialを使う．なので書き込む時にGPIO3とGPIO1にそれぞれ接続した状態のままだと書き込みに失敗する．
- PCとESP32をシリアル通信しながらGPSまたは無線機をESP32と接続してシリアル通信する際は，以下のように接続する．逆だと通信できなかった．

	|通信区間|変数名|
	|:---:|:---:|
	|PC - ESP32|Serial|
	|GPS or Wireless - ESP32|Serial2|

## I2C通信
- ESP32でのI2Cのピンのアサインは以下のようになっている

|ピン名|ピン番号|
|:---:|:---:|
|SDA|21|
|SCL|22|
- プルアップ抵抗を挟むか挟まないかはセンサモジュールの仕様による
	- 今回使うMPU6050やME280のモジュールにはプルアップ抵抗があるので外部でプルアップする必要もないし，ESP32側からも何もしなくてもいい
	- もしプルアップをESP32側から設定するならpinMode()の第二引数に"INPUT_PULLUP"を入れる
### Wire.hの扱いについて
- ArduinoだとWire.begin()の引数には何も入れなかったが，ESP32では第一引数にSDAピン番号，第二引数にSCLピン番号を入れる

## アナログ入力
- A1，A2，A8，A9を除いたA0-A19の計1個のピンがアナログ入力として使用可能
- A〇っていう番号はアナログ入力ピン番号であり，I/Oピン番号とは異なることに注意
	- マイコンボード左側のピン(USBポートを手前にしたとき)
	
	|アナログ入力ピン番号|I/Oピン番号|
	|:---:|:---:|
	|A0|36|
	|A3|39|
	|A6|34|
	|A7|35|
	|A4|32|
	|A5|33|
	|A18|25|
	|A19|26|
	|A17|27|
	|A16|14|
	|A15|12|
	|A14|13|
	
	- マイコンボード右側のピン(USBポートを手前にしたとき)
	
	|アナログ入力ピン番号|I/Oピン番号|
	|:---:|:---:|
	|A10|4|
	|A11|0|
	|A12|2|
	|A13|15|
	
	- ピン番号は順番に並んでいるわけではないことに注意
	- AnalogRead()の引数にはアナログ入力番号，I/Oピン番号のどちらを入れてもいい
- ESP32においてAnalogRead()のレンジは0-4095であることに注意(Arduinoでは0-1023)
	
	- 精度はESP32のほうがいい
- I/Oピンの基準電圧は3.3V
	
	- 5V出力のアナログ出力のセンサを用いる場合はセンサとマイコンボードの間にレベル変換ICを挟む必要がある
- A11(I/O0)のピンを使うとき，ピンに接続したままプログラムを書き込もうとするとエラーがでる
	
	- たぶんSerialの時と同じ現象
## タイマー割込み
- **ESP32はまさかの"MsTimer2"ライブラリが使えない！！**
- ライブラリは特になく何もインクルードしなくても以下の方法でタイマーを実装できる
	1. グローバル変数を準備
		- 以下のコードをメイン関数の上に書く
		```c++
		volatile int timeCounter1;
		hw_timer_t *timer1 = NULL; 
		portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
		```
		
	2. 使用タイマーの指定と初期化
		- ESP32では4つのタイマーを使用することができる．タイマーの使用に先立って，まずtimerBegin関数で使用するタイマーの番号などを指定して初期化処理を行う．この関数はesp-hal-timer.h内で次のようにプロトタイプ宣言されている．
		```c++
		hw_timer_t * timerBegin(uint8_t timer, uint16_t divider, bool countUp);
		```
			- 第一引数：使用するタイマー番号(0~3)
			- 第二引数：プリスケーラー（1マイクロ秒ごとにインクリメントさせたいなら80を指定）
			- 第三引数：割込カウンターのインクリメント（カウントアップ）指定
		- ちなみにプリスケーラーとは周波数をカウントするもので，この関数では1usごとにインクリメント(プラス1する)なら80という数字を入れるらしい
		- 例えばこんな感じ
		```c++
		timer = timerBegin(0, 80, true);
		```
		
	3. 割込み処理関数(ISR)の指定
		- ISR関数を結びつける
		- 以下のように関数が定義されている
		```c++
		void timerAttachInterrupt (hw_timer_t * timer, void (* fn)(void), bool edge);
		```
			- 第一引数：初期化されたタイマー設定用のポインター
			- 第二引数：ISR関数のアドレス
			- 第三引数：エッジ割込指定
		- onTimer()という名前の割込処理関数を記述するのであれば，次のようなコードになる．第1引数には，グローバル変数で定義したタイマー設定用ポインターを指定する．
		```c++
		timerAttachInterrupt(timer, &onTimer, true);
		```
	4. タイマーの動作間隔の指定
		- タイマーの動作間隔を指定する
		- 以下のように関数が定義されている
		```c++
		void timerAlarmWrite (hw_timer_t * timer, uint64_t alarm_value, bool autoreload);
		```
			- 第一引数：初期化されたタイマー設定用のポインター
			- 第二引数：割込みが発生する間隔(単位は**マイクロ秒**)
			- 第三引数：カウンターのリロード指定（定期的に割り込みを生成させる）
		- 1秒間隔で割込みを行う場合は以下のようになる
		```c++
		timerAlarmWrite(timer, 1000000, true);
		```
	5. タイマーの有効化
		- timerというポインタを引数に入れて
		```c++
		timerAlarmEnable(timer);
		```
	6. ISR関数を構成
		- とりあえず以下のような関数を定義する
		```c++
		void IRAM_ATTR onTimer1(){
  			portENTER_CRITICAL_ISR(&timerMux);
  			timeCounter1++;
  			portEXIT_CRITICAL_ISR(&timerMux);
		}
		```
		- interruptCounterという変数がインクリメントすることでloop関数に割込み処理があることを通知する．
	7. loop関数における割込み検知処理
		- ISR関数で割込み通知が来るため，カウンターをクリアして必要な割込み処理を行う
		- 以下のようにコードを書く
		```c++
		void loop() {
  			if (timeCounter1 > 0) {
    			portENTER_CRITICAL(&timerMux);
    			timeCounter1--;
    			portEXIT_CRITICAL(&timerMux);
				/*以下，割込み処理の内容*/
			}
		}
		```
- 以上がESP32で割込みを行う手順
- 面倒…

## SD
- Arduino同様に"SD.h"をインクルードして使う
- SPI通信で行う
- CS(チップセレクト：通信相手を指定する信号線)は多分どこのGPIOでもいい(要調査)
	- 動作確認ではGPIO5を使用
- Arduinoでは生成するファイル名のみでSDライブラリ内のopen()の実行ができるが，ESP32ではファイル名と合わせてディレクトリも明示しないといけない
	- ファイル名の横に **/** を付ける

```c++
const char* f_name = "/test.txt";
File myFile;
```
- Arduinoではopen()の第2引数にはFILE_READかFILE_WRITEを入れる
- ESP32でも読み出しはFILE_READで良いが，書き込みはFILE_APPENDとする必要がある
	- FILE_WRITEだと1行しか書き込んでくれない
	- ESP32ではFILE_WRITEは常に1行目に書き込まれていく仕様らしい
	
```c++
myFile = SD.open(f_name,FILE_APPEND);
```
- 接続のピンは以下の通り

|信号線の種類|SD側のピン|ESP32のピン|
|:---:|:---:|:---:|
|MOSI|3|23|
|MISO|7|19|
|CS(SS)|2|5|
|SCK|5|18|
|Vcc|4|3.3|
|GND|6|GND|

- 実際に実装するときの要求として直接テキストファイルに書き込むんじゃなくて，バイナリファイル(.bin)でとりあえず保存してほしい
- 文字列をファイルの中に書き込む処理は結構遅く，高サンプリングレートだと処理が追い付かない
- 一方バイナリ(0 or 1)でデータ列を書き込むのはめっちゃ早い
- もちろんそのままだと人間が読めないので，ロケットからデータを回収した後にテキストファイルに変換する
- byte型で書き込む方法はprint()ではなく，write(buf,len)を使う
	- bufはbyte型のデータ配列，lenはデータ配列の要素数
- またセンサから得た数値が実数のままbyte型に変換できない(離散化するので実数のどこかが切り落とされる．まぁプログラミングで扱う実数もすでに離散化されてるけど)
- なので実数値に10の有効数字乗をかけた整数値に直す
	- このときintじゃなくてint32_tのデータ型使って
	- intは16bitでint32_tは32bitで格納できる容量が違う
- int32_t型の整数から4つのbyteに分割して書き込む
	- 分割するのはビットシフトとかマスクという演算があるので各自調べること
## サーボモーター
- Arduinoにはサーボモーターの制御をするライブラリであるServoが用意されている
- が，案の定ESP32では使えない(Servoライブラリを利用するとコンパイルエラーが出る)
- 色々ESP32用のライブラリがある中，[こいつ](https://github.com/takashiAg/ESP32_servo)がいい感じに使えたのでこれをダウンロードしてインクルードする
- 最初にESP_servoのオブジェクト宣言をする
- 関数の説明
	- ``` init(uint_8 pin, uint_8channel); ```
		- 初期化関数
		- pinはサーボモーターを制御するGPIOピン
		- channelはESP32のPWMに割り当てられているチャンネル(0～15の整数)
	- ``` write(uint16_t degree); ```
		- サーボモーター角度指定関数
		- degreeは角度(°)
- 初期化関数だけ少し違うが，それ以外はArduinoのServoと同様に使える
- サーボモーターを駆動してからは必ず1秒以上のdelayを設けること
	- delayがないと電気的にサーボモーターに大きな負荷がかかる
