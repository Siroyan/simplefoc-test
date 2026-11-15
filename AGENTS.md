# 引き継ぎ：SimpleFOC実機試験

## 最初に読むこと

- 対象はこの `simplefoc-test`。隣の `spresense-feetech` は別プロジェクト。
- ユーザーは2026-11-15に「一旦実験は中断」と指示。2026-11-16の依頼は、
  引き継ぎ文書と現在のコードのプッシュ。実機実験を再開する依頼ではない。
- 再開の指示があるまで、書き込み・リセット・駆動コマンドの送信を行わない。
  静的解析やビルドは可能。別エージェントへの自動送信・起動も今回の依頼に含まれない。
- 最終実験後は出力OFFをログで確認。現在の実物の接続・電源状態は未確認。
  ファームウェアは起動だけでは回らないが、`t` / `o` は軸を動かす。
- まだ試験用ソフト。安定した3秒間の連続回転は達成していない。

## 目的と現状

V2806-KV400をB-G431B-ESC1上のSimpleFOCで制御し、AS5047PのABIを使う。
ABI読出し、6PWM、低電圧での方向・電気角合わせ、実際の軸の回転までは確認済み。
閉ループはカクカクし、オープンループはユーザーの観察で「前より滑らか」。
両方とも速度保護により目標の3秒より前に停止する。
実際の瞬間速度変動と速度推定のスパイクは切り分け未完了。

## ハードウェア・配線

- モーター：SunnySky V2806-KV400、400rpm/V。仕様書に12N14P、極対数 **7**。
- ESC：ST B-G431B-ESC1、内蔵ST-LINK/V2-1経由で書き込み・UART。
- センサー：AS5047P ADAPTERBOARD。SPIではなく **ABI** を使う。
- 電源装置：11.1V、最大約10Aまで可能だが試験時の電流制限は **0.3A**。
  この制限を無断で上げて試験しない。低電圧駆動の指令値は相電流上限を保証しない。
- ユーザー報告：試験時CV維持。0.7V向き合わせ時の電源電流は約0.2Aとの記憶。
  ソフトから電源電圧・電流を測定しているわけではない。
- 初期に主電源を逆接続し、実測0.8V・0.3AでCCになった。修正後は11.1V・約0.1Aで動作。
  以後書き込み・ABI・駆動は動作したが、逆接続による潜在的影響を検証したわけではない。
- 磁石：中心ずれ0.5mm以内、隙間3mm以内とのユーザー報告。
  磁石・固定・電源・配線の実物をエージェントが目視確認したわけではない。

| ESC J8 | センサー側 |
| --- | --- |
| 5V | 5V / P1-1（5V動作設定） |
| GND | GND / P1-8 |
| A+/H1 = PB6 | A / P1-10 |
| B+/H2 = PB7 | B / P1-9 |
| Z+/H3 = PB8 | I/PWM / P1-11（Index） |

5V設定はアダプター資料上R1実装・R2未実装が標準。実装状態は未目視確認。
以前CSnプルアップ、CLK/MOSIプルダウンを案内したが、ユーザーによる実施確認はない。
J8には信号入力の抵抗・クランプ回路がある。5V信号をMCUの任意ピンへ直結しない。

6PWMの順序：U高/低=PA8/PC13、V高/低=PA9/PA12、W高/低=PA10/PB15。
TIM1_CH1/1N、2/2N、3/3N。PWM=25kHz、dead_zone=0.02。
UARTは標準Serial（TX=PB3、RX=PB4）、115200baud。
PA5/PA6/PA7はこの基板では電流検出に使われるので、一般的なSPIピンとして流用しない。

## ビルド環境と既知の問題

`platformio.ini` に固定：

- platform = ststm32@20.0.0
- platform_packages = platformio/framework-arduinoststm32@4.21200.0（Arduino core 2.12.0）
- lib_deps = askuric/Simple FOC@2.4.0
- lib_archive = no
- upload_protocol = stlink

core 3.0.0ではSimpleFOCのenableTimerClock引数型が合わずコンパイル失敗。
core 2.11向けパッケージは当時レジストリで取得不可、取得できた2.12.0で固定した。
lib_archive既定値では汎用weak版_configure6PWMがリンクされ、driver.init()が失敗した。
`lib_archive=no`でSTM32のstrong実装がリンクされ、TIM1初期化成功。
依存ライブラリやグローバルなフレームワークのソースは変更していない。

```sh
# pioがPATHにない場合の、このMacのCLI
/Users/siroyan/.platformio/penv/bin/pio run
# 以下は実機再開の指示と接続条件の確認後のみ
/Users/siroyan/.platformio/penv/bin/pio device list
/Users/siroyan/.platformio/penv/bin/pio run -t upload
/Users/siroyan/.platformio/penv/bin/pio device monitor -b 115200
```

最終試験時のポートは `/dev/cu.usbmodem1103`、VID:PID=0483:374B。
ポート名は再検出すること。最初はUSBが見えなかったが、後にST-LINK・VCPとも認識し、
書き込み・verify・シリアル取得に成功した。ドライバー導入は行っていない。

## 現在のsrc/main.cpp

- 起動時6ゲートLOW、PWM初期化後driver.disable()。駆動開始はシリアル操作のみ。
- 1起動につき1試験。失敗しても再試行にはリセットが必要。
- `t`：閉ループ速度制御。向き合わせ後、1rad/s（約9.55rpm）、上限0.5V、3秒。
- `o`：オープンループ。向き合わせ後、その終端磁界から速度を800msで0→3rad/s
  （約28.65rpm）へ上げ、0.7V一定で最大3秒。速度PIDを使わない。
- `s`：出力無効化。向き合わせ中も監視。motor.init()の約1秒のブロッキング中は即応しない。
- 保護：観測速度の絶対値5rad/s超、非有限速度、3秒経過、1.2秒経過時の開始点からの
  変位0.03rad未満で停止。相電流・温度・VBUSによるソフト保護は未実装。
  この変位判定は継続的な停止検知や反転の判定にはなっていない。
- PPR=1024仮設定、4逓倍4096counts/turn。Indexは立ち上がり回数を観測するだけ。
  原点補正・Index探索は行わない。ABIの起動角は絶対角ではない。
- 閉ループ：P=0.08、I=0.15、D=0、出力ランプ0.3V/s、速度LPF=0.05s、downsample=4。
- オープンループの速度保護は **Encoderの実測推定速度＋0.05s LPF**。
  motor.shaft_velocityはオープンループでは指令速度になるので保護に使っていない。
- getVelocity()は内部状態を更新する。閉ループ時に独立観測用として二重に呼ばない。
- CSV（5Hz）：ms,running,A_edges,B_edges,index,angle_rad,velocity_rad_s,Uq_V。
  A/Bは各両エッジの累積値。Uqは指令値であり電圧の実測ではない。
  停止後はUq欄を0表示する。5Hzログには瞬間的な速度ピークが残らない。

### 独自の向き合わせ

標準initFOCの1電気回転・2ms/stepでは移動不足の極対数判定となった。
現在はalignSlowly()で0.7Vを用い、初期700ms待機、3電気回転を4ms/stepで往復、
各終端700ms待機。機械角は片道約154度、往復で約14秒＋motor.init()。
往路と復路の符号一致を確認し、各移動量から1電気回転あたりの誤差を計算、
**0.5電気rad以内**の場合に方向・電気角オフセット・pp_check_resultを設定する。
単に標準チェックを無効化したのではないが、独自実装であり妥当性の再検討は可能。
`t`のinitFOCはこの方向・オフセットを使うため標準の再測定をスキップする。
`o`も同じ独自チェックを通す。安定回転はまだ実証できていない。

## 実測結果（2026-11-15）

| 試験 | 結果 |
| --- | --- |
| 手動で約1回転 | A=2097、B=2093エッジ増、Index=1。ユーザーは少し回しすぎたと報告。1024PPRと整合するが厳密校正ではない |
| 標準向き合わせ0.3/0.5/0.7/0.9V | 推定極対数10.81/8.27/7.82/7.71、チェックで停止 |
| 低速向き合わせ・1電気回転 | 往路0.81915/復路0.81301rad、判定失敗 |
| 低速向き合わせ・3電気回転 | 往路2.61084/復路2.60777rad、判定成功 |
| 閉ループ初回 | 約25度動き、約1秒で速度保護停止 |
| 閉ループゲイン調整後 | 約39.6度動き、約2.2秒で速度保護停止。実物もカクカクとの報告 |
| オープンループ・無整列開始 | 開始磁界への引き寄せで速度保護停止、実装修正済み |
| オープンループ・整列後開始 | 約87.9度動き、約1秒で速度保護停止。ユーザーは「前より滑らか」と報告 |

最後の整列後オープンループの角度は-0.30066→1.23332rad。
直前の向き合わせ測定は2.63231/2.59243rad。
ユーザーは試験中CV維持と報告。ただし各試験全期間の電源ログは存在しない。

## 次の解析の候補（未検証の仮説）

1. 高速ログで生の角度差分、Encoder速度推定、LPF後速度、指令速度、Uqを比較する。
   速度停止時の値・直前の履歴を保存し、実速度上昇か推定スパイクか判別する。
   現状は「STOP: unexpected velocity」に停止時の速度値がない。
2. 独立した一定時間窓での角度差分速度とEncoder::getVelocity()を比較する。
   保護閾値を根拠なく上げたり外したりして回転成功と扱わない。
3. 低電圧でのコギング、駆動波形・デッドタイムの影響、電気角オフセットの妥当性、
   配線・磁石の固定・センサー信号を切り分ける。いずれも原因確定ではない。
4. 必要なら基板内蔵相電流検出を導入。ただしシャント・OPAMP・ゲイン・符号・ADC同期は
   回路図・ライブラリ実装で確認する。未実装の電流制限を効いていると説明しない。
5. PPRの厳密確認はIndex間のカウント計測等が候補。機械角の手回しは記録開始後に案内する。
   以前2回は記録開始前に回転が始まり測定を取り逃した。

電流制限・駆動電圧・速度上限を漫然と上げず、計測と根拠を先に整える。
再開時は電源・固定・接続とユーザーの実機準備を確認してから短時間試験にする。

## 資料と証跡

- README.md：試験手順と時系列結果。AGENTS.mdを現在の引き継ぎ状態として優先。
- docs/experiment-logs/：当時のシリアルログ・手回しCSVを保存。最終版より前のログも含む。
- モーター商品：https://akizukidenshi.com/catalog/g/g130949/
- モーター仕様：https://akizukidenshi.com/goodsaffix/V2806-KV400%20SpecificationV1%2020231023.pdf
- ESCマニュアル：https://www.st.com/resource/en/user_manual/dm00564746-electronic-speed-controller-discovery-kit-for-drones-with-stm32g431cb-stmicroelectronics.pdf
- ESC回路図：https://www.st.com/resource/en/schematic_pack/mb1419-g431cbu6-c01_schematic.pdf
- センサー基板資料：https://www.mouser.com/datasheet/2/588/AS5047P-TS_EK_AB_Operation-Manual_Rev.1.0-775823.pdf
- SimpleFOC：https://docs.simplefoc.com/encoder 、 https://docs.simplefoc.com/bldcdriver6pwm

Git originはhttps://github.com/Siroyan/simplefoc-test.git、ブランチmain。
ローカルGit作者：Siroyan / 1517053@ed.tus.ac.jp。グローバル設定は変更しない。
