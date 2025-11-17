"""
ShaderMotion encoder for ROS 2 TF data
Encodes robot link positions/orientations into color grid for virtual camera output
"""

import numpy as np
from typing import List, Tuple

class ShaderMotionEncoder:
    """ShaderMotion風のエンコーダー"""

    def __init__(self, grid_width=80, grid_height=45):
        """
        Args:
            grid_width: カラーグリッドの幅（デフォルト: 80）
            grid_height: カラーグリッドの高さ（デフォルト: 45）

        Note:
            60球体の場合: ヘッダー1 + データ120スロット = 121スロット必要
            80幅 × 45高 = 40スロット/行 × 45行 = 1800スロット利用可能（十分）
        """
        self.grid_width = grid_width
        self.grid_height = grid_height
        self.total_slots = (grid_width // 2) * grid_height  # 各スロットは2つの正方形

    def encode_frame(self, positions: np.ndarray, rotations: np.ndarray, seq: int = 0) -> np.ndarray:
        """
        60個の球体データをカラーグリッドにエンコード

        Args:
            positions: (60, 3) の位置配列 [x, y, z]
            rotations: (60, 4) のクォータニオン配列 [x, y, z, w]
            seq: シーケンス番号

        Returns:
            (height*block_size, width*block_size, 3) のRGB画像
        """
        # データをスロットに配置
        slots = []

        # ヘッダー: シーケンス番号（1スロット = 6値）
        seq_slot = self._encode_uint16(seq)
        slots.append(seq_slot)

        # 60個の球体データ（ロボット1台30個 × 2台）
        for i in range(60):
            # 位置 (x, y, z) → 各16bit → 3スロット分の6値
            pos = positions[i]
            pos_values = self._encode_position_16bit(pos)  # 6値

            # 回転 (x, y, z, w) → 各16bit → 4スロット分の8値
            rot = rotations[i]
            rot_values = self._encode_quaternion_16bit(rot)  # 8値

            # スロット1: pos_x(2), pos_y(2), pos_z(2)
            slot1 = pos_values  # 6値

            # スロット2: rot_x(2), rot_y(2), rot_z(2)
            slot2 = rot_values[:6]  # 6値

            # スロット3: rot_w(2), padding(4)
            slot3 = np.concatenate([rot_values[6:8], np.zeros(4)])  # 2値 + パディング4値

            slots.append(slot1)
            slots.append(slot2)
            slots.append(slot3)

        # カラーグリッドを生成
        grid = self._create_color_grid(slots)

        return grid

    def _encode_position_16bit(self, pos: np.ndarray) -> np.ndarray:
        """
        位置を16bitエンコード (3成分 → 6値)
        想定範囲: [-5, 5] m
        
        符号-絶対値形式 (14bit精度):
        bit15: 符号ビット (0=負, 1=正)
        bit14: 予約ビット (常に0)
        bit13-0: 絶対値 (0-16383, 14bit精度)
        
        利点:
        - 0付近の値が安定 (ビデオ圧縮での異常値を防止)
        - 絶対値の劣化は線形的
        - 14bit精度: 5m / 16384 ≈ 0.3mm の分解能
        
        ビット交互配置:
        high_byte: bit15(符号), bit13, bit11, bit9, bit7, bit5, bit3, bit1 (奇数ビット)
        low_byte:  bit14(0), bit12, bit10, bit8, bit6, bit4, bit2, bit0 (偶数ビット)
        """
        # 各成分を符号-絶対値形式で16bitに変換し、ビット交互配置で分割
        values = []
        for val in pos:
            # 符号と絶対値に分離
            sign = 1 if val >= 0 else 0
            abs_val = abs(val)
            abs_val = min(abs_val, 5.0)  # [-5, 5]にクリップ
            
            # 絶対値を14bitに変換 (0-16383)
            # 5m → 16383 にマッピング
            abs_14bit = int((abs_val / 5.0) * 16383.0)
            
            # bit15=符号, bit14=0(予約), bit13-0=絶対値
            val_16bit = (sign << 15) | (0 << 14) | abs_14bit
            
            # ビット交互配置: 奇数ビットと偶数ビットを分離
            high_byte = (
                ((val_16bit >> 15) & 0x01) << 7 |  # bit15(符号) → bit7
                ((val_16bit >> 13) & 0x01) << 6 |  # bit13 → bit6
                ((val_16bit >> 11) & 0x01) << 5 |  # bit11 → bit5
                ((val_16bit >>  9) & 0x01) << 4 |  # bit9  → bit4
                ((val_16bit >>  7) & 0x01) << 3 |  # bit7  → bit3
                ((val_16bit >>  5) & 0x01) << 2 |  # bit5  → bit2
                ((val_16bit >>  3) & 0x01) << 1 |  # bit3  → bit1
                ((val_16bit >>  1) & 0x01)         # bit1  → bit0
            )
            
            low_byte = (
                ((val_16bit >> 14) & 0x01) << 7 |  # bit14(0) → bit7
                ((val_16bit >> 12) & 0x01) << 6 |  # bit12 → bit6
                ((val_16bit >> 10) & 0x01) << 5 |  # bit10 → bit5
                ((val_16bit >>  8) & 0x01) << 4 |  # bit8  → bit4
                ((val_16bit >>  6) & 0x01) << 3 |  # bit6  → bit3
                ((val_16bit >>  4) & 0x01) << 2 |  # bit4  → bit2
                ((val_16bit >>  2) & 0x01) << 1 |  # bit2  → bit1
                ((val_16bit      ) & 0x01)         # bit0  → bit0
            )
            
            values.append(high_byte / 255.0)     # [0,1]に正規化
            values.append(low_byte / 255.0)      # [0,1]に正規化

        return np.array(values)  # 6値

    def _encode_quaternion_16bit(self, quat: np.ndarray) -> np.ndarray:
        """
        クォータニオンを16bitエンコード (4成分 → 8値)
        入力: [x, y, z, w] 範囲 [-1, 1]
        
        符号-絶対値形式 (14bit精度):
        bit15: 符号ビット (0=負, 1=正)
        bit14: 常に0 (予約ビット)
        bit13-0: 絶対値 (0-16383, 14bit精度)
        
        利点:
        - 0付近の値が安定 (パディング値問題の解消)
        - 絶対値の劣化は線形的
        - 14bit (16384段階) でも十分な精度
        
        ビット交互配置:
        high_byte: bit15(符号), bit13, bit11, bit9, bit7, bit5, bit3, bit1 (奇数ビット)
        low_byte:  bit14(0), bit12, bit10, bit8, bit6, bit4, bit2, bit0 (偶数ビット)
        """
        # 正規化
        norm = np.linalg.norm(quat)
        if norm > 0.0001:
            quat = quat / norm

        # 各成分を符号-絶対値形式で16bitに変換し、ビット交互配置で分割
        values = []
        for val in quat:
            # 符号と絶対値に分離
            sign = 1 if val >= 0 else 0
            abs_val = abs(val)
            abs_val = min(abs_val, 1.0)  # [-1,1]にクリップ
            
            # 絶対値を14bitに変換 (0-16383)
            abs_14bit = int(abs_val * 16383.0)
            
            # bit15=符号, bit14=0(予約), bit13-0=絶対値
            val_16bit = (sign << 15) | (0 << 14) | abs_14bit
            
            # ビット交互配置: 奇数ビットと偶数ビットを分離
            high_byte = (
                ((val_16bit >> 15) & 0x01) << 7 |  # bit15(符号) → bit7
                ((val_16bit >> 13) & 0x01) << 6 |  # bit13 → bit6
                ((val_16bit >> 11) & 0x01) << 5 |  # bit11 → bit5
                ((val_16bit >>  9) & 0x01) << 4 |  # bit9  → bit4
                ((val_16bit >>  7) & 0x01) << 3 |  # bit7  → bit3
                ((val_16bit >>  5) & 0x01) << 2 |  # bit5  → bit2
                ((val_16bit >>  3) & 0x01) << 1 |  # bit3  → bit1
                ((val_16bit >>  1) & 0x01)         # bit1  → bit0
            )
            
            low_byte = (
                ((val_16bit >> 14) & 0x01) << 7 |  # bit14(0) → bit7
                ((val_16bit >> 12) & 0x01) << 6 |  # bit12 → bit6
                ((val_16bit >> 10) & 0x01) << 5 |  # bit10 → bit5
                ((val_16bit >>  8) & 0x01) << 4 |  # bit8  → bit4
                ((val_16bit >>  6) & 0x01) << 3 |  # bit6  → bit3
                ((val_16bit >>  4) & 0x01) << 2 |  # bit4  → bit2
                ((val_16bit >>  2) & 0x01) << 1 |  # bit2  → bit1
                ((val_16bit      ) & 0x01)         # bit0  → bit0
            )
            
            values.append(high_byte / 255.0)
            values.append(low_byte / 255.0)

        return np.array(values)  # 8値

    def _encode_uint16(self, value: int) -> np.ndarray:
        """
        uint16を6個の[0,1]値にエンコード
        簡易版: 各ビットグループを分割
        """
        # 16ビットを6つに分割（各約2.67ビット）
        values = np.zeros(6)
        for i in range(6):
            shift = i * 3  # 3ビットずつ
            mask = 0x7  # 0b111
            bits = (value >> shift) & mask
            values[i] = bits / 7.0  # [0, 7] → [0, 1]
        return values

    def _values_to_colors(self, values: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        6個の[0,1]値を2つのRGB色に変換
        ShaderMotion方式: GRBGRB順序

        Args:
            values: (6,) の配列

        Returns:
            color1, color2: 各(3,) のRGB配列
        """
        # GRBGRBの順序で並べ替え
        # values[0] → G1, values[1] → R1, values[2] → B1
        # values[3] → G2, values[4] → R2, values[5] → B2

        color1 = np.array([values[1], values[0], values[2]])  # RGB
        color2 = np.array([values[4], values[3], values[5]])  # RGB

        return color1, color2

    def _create_color_grid(self, slots: List[np.ndarray], block_size: int = 8) -> np.ndarray:
        """
        スロットリストからカラーグリッド画像を生成
        YUV420圧縮対策：各値をグレースケール(R=G=B)で出力してY成分のみ使用

        1スロット = 16x8ピクセル領域
        6つの値を3x2グリッドで配置（各値は約5x4ピクセルブロック）
        ※ 2x3から3x2に変更：縦を長くすることでYUV420圧縮に強くする

        Args:
            slots: スロットのリスト（各スロットは6値）
            block_size: 各カラーブロックのピクセルサイズ (デフォルト8)

        Returns:
            (height*block_size, width*block_size, 3) のRGB画像
        """
        img_height = self.grid_height * block_size
        img_width = self.grid_width * block_size
        image = np.zeros((img_height, img_width, 3), dtype=np.float32)

        slot_index = 0
        for row in range(self.grid_height):
            if slot_index >= len(slots):
                break  # 外側ループを抜ける
                
            for col in range(0, self.grid_width, 2):  # 2列ずつ（1スロット = 16x8ピクセル）
                if slot_index >= len(slots):
                    break  # 内側ループを抜ける

                # スロットから6つの値を取得
                values = slots[slot_index]

                # 16x8領域に6つの値を2行3列で配置
                # 各セルは約5x4ピクセル（16/3 ≈ 5.3, 8/2 = 4）
                slot_y = row * block_size
                slot_x = col * block_size

                for i in range(6):
                    # 3x2グリッドでの位置（横3列×縦2行）
                    grid_row = i // 2  # 0, 1, or 2
                    grid_col = i % 2   # 0 or 1

                    # グレースケール値 (R=G=B) - YUV420でY成分のみ使用
                    gray = values[i]
                    color = np.array([gray, gray, gray])

                    # ブロック位置とサイズ
                    cell_height = block_size // 3  # 約2.67ピクセル → 2ピクセル（切り捨て）
                    cell_width = (block_size * 2) // 2  # 8ピクセル

                    y1 = slot_y + grid_row * cell_height
                    y2 = y1 + cell_height
                    x1 = slot_x + grid_col * cell_width
                    x2 = x1 + cell_width

                    # 境界チェック
                    y2 = min(y2, img_height)
                    x2 = min(x2, img_width)

                    image[y1:y2, x1:x2] = color

                slot_index += 1

        return image
