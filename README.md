# ベンジョミン(MCPCナノコンコンテスト優秀賞作品)

# 作成するに至った背景

- 開いているトイレを知りたい<br>
![hatena](https://user-images.githubusercontent.com/75054606/146580015-057a9326-fd7c-44c1-a5bd-04c4d65468a7.png)
<br>

- こもりスマホやめて<br>
![yamete](https://user-images.githubusercontent.com/75054606/146580010-7d6501f4-f8e2-4f26-91d7-74e74845bd30.png)
<br>




# Leafony構成(上から順に)
- AI02 SP&PIR
- AZ01 USB
- AP01 AVR MCU
- AC02 BLE Sugar
- AV01 CR2032

# システム構成図
![完成版](https://user-images.githubusercontent.com/75054606/139001432-5a123e65-ea09-4c4e-bc89-8ba5264db898.png)
<!-- 改ページ -->
<div style="page-break-before:always"></div>

- Leafony
  - 今回はブロードキャスト型の通信を利用する
    1. 人感センサから取得した値を発信する
    2. 1.が終えたらiBeaconをを発信する
    3. スリープ状態に遷移する
    4. 設定した秒数のスリープを経て、 1.に戻る


- Raspberry Pi
  - ゲートウェイの役割を担っている
  - Pythonのライブラリであるbluepyを用いてLeafonyの発信した人感センサの値を受信する
  - 受信した値を元に在室状況を判定し、Firebaseに登録する
  - 在室の判定には閾値を設定している

- Firebase
  - 在室の状態を保持している
  - リアルタイム更新が可能

- Swift
  - アプリを構成している
  - iBeaconを受信した際の処理を行なっている
    1. iBeacon受信
    2. Firebaseの入室時間と現在時刻を比較
    3. 2.の差分が５分以上なら通知をする

  
# 拡張性
![a](https://user-images.githubusercontent.com/75054606/138926716-30132090-c280-4a89-88cc-80af34347354.png)

- 規模の大きいトイレでもLeafonyを増やすだけで対応できる
- 会社、 ショッピングモールなど場所を問わず導入可能である

<!-- 改ページ -->
<div style="page-break-before:always"></div>

# 今後の課題
- 通知を出せるのはアプリを入れている人に限られてしまう
  - 解決案: 会社やショッピングモールなどの公式アプリに付随する機能として導入
- iOS限定
  - 解決案: DartのフレームワークであるFlutterなどのクロスプラットフォーム開発可能な言語で開発することによってAndroidでも使用可能

# あとがき
 こもりスマホは、本当に困っている人にとってしてみればとても迷惑な行為である．また、こもりスマホをしている人自身の健康を害する危険性もある．誰にとってもメリットはないため、これらの問題を今回開発したベンジョミンが解決の糸口へ繋がることを願う．







