# InteractiveMD

**Version 1.0.2**

## 概要

DCモータをSPI通信を介して速度制御するシステム。   
内部のPID制御器により２系統のモータをそれぞれ速度及び電流制御を行う。

## 使用MCU

- [STM32F303K8](https://www.st.com/ja/microcontrollers-microprocessors/stm32f303k8.html)

## 使用機材

テスト環境での使用機材を次に示す。

### 電流センサ

- ACS712ELCTR-05B(電圧出力式電流センサ)

### 速度センサ

- 直交型2相ロータリエンコーダ   

### DCモータ

- 12V5Aクラス ブラシドDCモータ(ギア比 50:1)

### モータドライバ

- MC33926(フルNch 12V3A)    
    2線(PWM,DIR)により制御する方式(サインマグニチュードPWM方式)     
    PWM周波数:  20 kHz(固定)

### 通信インターフェイス

- [MCP2210](https://www.microchip.com/en-us/product/MCP2210)    
    USB-SPI変換用IC 

## 使用環境

### MCU側

- STM32CubeIDE  Version: 1.7.0  
    HAL-API based   

### ホスト側

- Ubuntu 20.04.3 LTS    
    Linux HID APIを使用

## 導入方法

STM32CubeIDEより、`File -> Import...`を選択。   
ウィザードは`Git -> Projects from Git`を選択。  

![step1](https://github.com/TaiyouKomazawa/imd_docs_images/blob/main/step1.jpeg?raw=true)

次に、リモートから本プロジェクトをクローンするので`Clone URI`を選択する。   

![step2](https://github.com/TaiyouKomazawa/imd_docs_images/blob/main/step2.jpeg?raw=true)

`Location`の`URI`に本リポジトリのURLを指定する。  
``` 
    https://github.com/TaiyouKomazawa/InteractiveMD.git
```     

![step3](https://github.com/TaiyouKomazawa/imd_docs_images/blob/main/step3.jpeg?raw=true)

ここではブランチを選択するが、mainだけでも問題ない。  

![step4](https://github.com/TaiyouKomazawa/imd_docs_images/blob/main/step4.jpeg?raw=true)

インポート先のディレクトリを選ぶ。STM32CubeIDEのワークスペース以下に置くと管理が楽。    
ここで、必要なサブモジュールがあるので`Clone submodules`には**必ずチェックを入れる。**

![step5](https://github.com/TaiyouKomazawa/imd_docs_images/blob/main/step5.jpeg?raw=true)

ここで設定することは特にない。

![step6](https://github.com/TaiyouKomazawa/imd_docs_images/blob/main/step6.jpeg?raw=true)

最後に、`Projects`のパスに間違いがないか等を確認したら、`Finish`を押すとインポートされる。    

![step7](https://github.com/TaiyouKomazawa/imd_docs_images/blob/main/step7.jpeg?raw=true)

特に問題がなければ、プロジェクトエクスプローラに本プロジェクトのファイルツリーが表示される。    
あとは、プロジェクトを開いてビルドなり実行(書き込み)が無事完了すれば導入完了です。

## ライセンス

本プロジェクトは、BSD 3-Clause ライセンス下で提供されます。

