# 参考 https://qiita.com/c60evaporator/items/ed2ffde4c87001111c12

from bluepy import btle # BLE関連
import traceback # エラー吐かせる
import time
from db import update_db, get_state # 自作関数



class ScanDelegate(btle.DefaultDelegate):
    def __init__(self):  # コンストラクタ
        btle.DefaultDelegate.__init__(self)

    def handleDiscovery(self, dev, isNewDev, isNewData):  # スキャンハンドラー

        if dev.addr == '14:b4:57:a0:cb:4a':
            try:
                data = dev.scanData[255]
                state = False

                # ibeaconを取得した場合はdata[0]に16進数の4Cが格納されているのでその対策  
                if data[0] == 76:
                    raise ValueError
                    

                if(data[0] == 0):
                    # data[0] == 0　は負の数なので-1をかける
                    # roundで丸める
                    ir = round(((data[1]*100)+((data[2]*100))/256)*(-1))
                else:
                    ir = round((data[1]*100)+((data[2]*100))/256)
                
                print(ir)

                # 閾値
                if ir > 0:
                    state = True
                
                # fire base側のstateのを取得
                db_state = get_state(dev.addr)

                # fire base側とこっちの状態が違う場合のみdbを更新
                if db_state != state:
                    # db更新
                    update_db(dev.addr, state)

            except:
                # エラーを吐かせる
                # traceback.print_exc()
                pass

                 
scanner = btle.Scanner().withDelegate(ScanDelegate())

while True:
    
    try:
        scanner.scan(1.0)
        # time.sleep(1)
    except:
        traceback.print_exc()
        pass