import firebase_admin # firebase関連
from firebase_admin import credentials
from firebase_admin import db
import datetime
import os
from dotenv import load_dotenv # .env
load_dotenv()
CREDENTIAL = os.getenv('CREDENTIAL')
DBURL = os.getenv('DBURL')
UID = os.getenv('UID')

# 認証, 初期化
cred = credentials.Certificate(CREDENTIAL)
firebase_admin.initialize_app(cred, {
    'databaseURL': DBURL,
    'databaseAuthVariableOverride': {
        'uid': UID
    }
})

def update_db(address, state):
    # マイコンを選択 例'14:b4:57:a0:cb:4a'
    users_ref = db.reference(address)
    # 現在時刻
    dt_now = datetime.datetime.now()
    updates1 = {}
    updates2 = {}
    updates1['/state'] = state
    updates2['/updated_at'] = str(dt_now)
    users_ref.update(updates1)
    users_ref.update(updates2)
    print(users_ref.get())

def get_state(address):
    # マイコンを選択 例'14:b4:57:a0:cb:4a'
    state = db.reference(str(address)+'/state')

    return state.get()

    


# # テスト用
# if __name__ == '__main__':
#     update_db('14:b4:57:a0:cb:4a', True)
    
    
    

    


