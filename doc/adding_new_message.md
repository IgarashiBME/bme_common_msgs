# 新しいメッセージの追加手順

このドキュメントでは、bme_common_msgsパッケージに新しい.msgファイルを追加する手順を説明します。

## 1. メッセージファイルの作成

### 1.1 ファイルの配置

`msg/`ディレクトリに新しい`.msg`ファイルを作成します。

```bash
touch ~/ros2_ws/src/bme_common_msgs/msg/NewMessage.msg
```

### 1.2 命名規則（重要）

ROS2 Jazzyでは厳格な命名規則が適用されます。違反するとビルドエラーになります。

#### ファイル名
- **PascalCase**を使用（単語の先頭を大文字）
- アンダースコア(`_`)は使用不可

```
# 正しい例
PVT.msg
AutoLog.msg
MyNewMessage.msg

# 間違い例（ビルドエラーになる）
Nav_PVT.msg      # アンダースコア不可
auto_log.msg     # 小文字始まり不可
NAVPVT.msg       # 全て大文字は避ける
```

#### フィールド名
- **snake_case**を使用（小文字とアンダースコア）
- 大文字は使用不可

```
# 正しい例
uint32 itow
int32 rel_pos_n
float64 heading_rad

# 間違い例（ビルドエラーになる）
uint32 iTOW          # 大文字不可
int32 relPosN        # キャメルケース不可
float64 HeadingRad   # 大文字不可
```

## 2. メッセージ定義の記述

### 2.1 基本構造

```
# メッセージの説明コメント
# UBXプロトコル対応の場合はクラスIDとメッセージIDを記載

# 定数定義（オプション）
uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 7

# フィールド定義
型 フィールド名    # コメント（単位、説明）
```

### 2.2 使用可能なデータ型

#### プリミティブ型

| 型 | 説明 | 範囲 |
|----|------|------|
| bool | 真偽値 | true/false |
| int8 | 符号付き8ビット | -128 〜 127 |
| uint8 | 符号なし8ビット | 0 〜 255 |
| int16 | 符号付き16ビット | -32768 〜 32767 |
| uint16 | 符号なし16ビット | 0 〜 65535 |
| int32 | 符号付き32ビット | -2^31 〜 2^31-1 |
| uint32 | 符号なし32ビット | 0 〜 2^32-1 |
| int64 | 符号付き64ビット | -2^63 〜 2^63-1 |
| uint64 | 符号なし64ビット | 0 〜 2^64-1 |
| float32 | 単精度浮動小数点 | |
| float64 | 倍精度浮動小数点 | |
| string | 文字列 | |

#### 配列型

```
# 可変長配列
int32[] data

# 固定長配列
int32[10] fixed_data

# 上限付き可変長配列
int32[<=100] bounded_data
```

#### 他パッケージの型

```
# std_msgsの型
std_msgs/Header header

# builtin_interfacesの型
builtin_interfaces/Time stamp

# 同一パッケージ内の型
PVT nav_data
```

### 2.3 UBXプロトコル対応メッセージの例

```
# NAV-EXAMPLE (0x01 0x99)
# サンプルメッセージの説明

uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 153

# UBX Protocol Fields
uint32 itow             # GPS週内時刻 [ms]
int32 value_a           # 値A [単位]
int32 value_b           # 値B [単位]
uint8 flags             # フラグ (X1 bitfield)
                        # bit0: flag_a - フラグAの説明
                        # bit1: flag_b - フラグBの説明
                        # bits3..2: mode - モード (0=off, 1=on, 2=auto)
uint32 accuracy         # 精度 [mm]

# Custom calculated fields (not in UBX spec)
uint8 fix_status        # RTK状態 (0:なし, 1:Float, 2:Fixed)
float64 value_converted # 変換済みの値
```

### 2.4 UBXデータ型マッピング

| UBX型 | ROS2型 | 備考 |
|-------|--------|------|
| U1 | uint8 | 符号なし1バイト |
| I1 | int8 | 符号付き1バイト |
| U2 | uint16 | 符号なし2バイト |
| I2 | int16 | 符号付き2バイト |
| U4 | uint32 | 符号なし4バイト |
| I4 | int32 | 符号付き4バイト |
| X1 | uint8 | ビットフィールド（コメントで各ビットを説明） |
| X2 | uint16 | ビットフィールド |
| X4 | uint32 | ビットフィールド |
| R4 | float32 | 単精度浮動小数点 |
| R8 | float64 | 倍精度浮動小数点 |

## 3. CMakeLists.txtの編集

新しいメッセージファイルを`rosidl_generate_interfaces`に追加します。

```cmake
# Generate ROS2 message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AutoLog.msg"
  "msg/MavModes.msg"
  "msg/PVT.msg"
  "msg/RELPOSNED.msg"
  "msg/UTMHP.msg"
  "msg/NewMessage.msg"    # ← 追加
  DEPENDENCIES std_msgs builtin_interfaces
)
```

### 新しい依存パッケージが必要な場合

他のパッケージの型を使用する場合は、依存関係を追加します。

```cmake
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)  # ← 追加

# Generate ROS2 message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ...
  DEPENDENCIES std_msgs builtin_interfaces geometry_msgs  # ← 追加
)
```

## 4. package.xmlの編集（必要な場合）

新しい依存パッケージを追加した場合は、`package.xml`も編集します。

```xml
<depend>std_msgs</depend>
<depend>builtin_interfaces</depend>
<depend>geometry_msgs</depend>  <!-- 追加 -->
```

## 5. ビルドと確認

### 5.1 ビルド

```bash
cd ~/ros2_ws
colcon build --packages-select bme_common_msgs
source install/setup.bash
```

### 5.2 メッセージの確認

```bash
# メッセージ一覧の確認
ros2 interface list | grep bme_common_msgs

# メッセージ定義の確認
ros2 interface show bme_common_msgs/msg/NewMessage
```

## 6. チェックリスト

新しいメッセージを追加する際は、以下を確認してください：

- [ ] ファイル名がPascalCaseになっている
- [ ] フィールド名がsnake_caseになっている
- [ ] 各フィールドにコメント（単位、説明）がある
- [ ] ビットフィールドの場合、各ビットの意味をコメントで説明している
- [ ] CMakeLists.txtにファイルを追加した
- [ ] 新しい依存パッケージがある場合、CMakeLists.txtとpackage.xmlに追加した
- [ ] ビルドが成功する
- [ ] `ros2 interface show`で定義が正しく表示される

## 7. よくあるエラーと対処法

### エラー: invalid field name

```
NameError: 'iTOW' is an invalid field name. It should have the pattern '^(?!.*__)(?!.*_$)[a-z][a-z0-9_]*$'
```

**原因**: フィールド名に大文字が含まれている
**対処**: snake_caseに変更（例: `iTOW` → `itow`）

### エラー: invalid message name

```
InvalidResourceName: 'Auto_Log' is an invalid message name. It should have the pattern '^[A-Z][A-Za-z0-9]*$'
```

**原因**: ファイル名にアンダースコアが含まれている
**対処**: PascalCaseに変更（例: `Auto_Log.msg` → `AutoLog.msg`）

### エラー: KeyError processing template

```
KeyError processing template 'struct.idl.em': 'time'
```

**原因**: `time`型を直接使用している
**対処**: `builtin_interfaces/Time`に変更

## 8. 参照資料

- UBXプロトコル仕様書: `references/`ディレクトリ
- ROS2メッセージ定義: https://docs.ros.org/en/jazzy/Concepts/About-ROS-Interfaces.html
