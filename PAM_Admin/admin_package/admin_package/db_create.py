import mysql.connector

class DatabaseCreate:
    def __init__(self, host, user):
        self.host = host
        self.user = user
        self.db_name = "ros_final"
        self.cur = None
        self.conn = None
        self.password = "your_password"

    def connect_database(self, db_name=None):
        if db_name is None:
            db_name = self.db_name
        
        try:
            self.conn = mysql.connector.connect(
                host=self.host,
                user=self.user,
                database=db_name,
                password=self.password
            )
            self.cur = self.conn.cursor()
        except mysql.connector.Error as err:
            if err.errno == mysql.connector.errorcode.ER_BAD_DB_ERROR:
                self.create_database(db_name)
                self.connect_database(db_name)
            else:
                print("Failed to connect to database: {}".format(err))
                exit(1)

    def create_database(self, db_name):
        try:
            self.conn = mysql.connector.connect(
                host=self.host,
                user=self.user,
                password=self.password
            )
            self.cur = self.conn.cursor()
            self.cur.execute("CREATE DATABASE IF NOT EXISTS {}".format(db_name))
            print("Database {} created successfully.".format(db_name))
        except mysql.connector.Error as err:
            print("Failed creating database: {}".format(err))
            exit(1)
        finally:
            if self.cur:
                self.cur.close()
            if self.conn:
                self.conn.close()
    
    def create_table(self):
        self.connect_database()  # 데이터베이스 연결
        try:
            self.cur.execute("""
                CREATE TABLE IF NOT EXISTS Scripts (
                    name CHAR(255),
                    script TEXT
                )
            """)
            print("Table 'Scripts' created successfully.")

            self.cur.execute("""
                CREATE TABLE IF NOT EXISTS EventLog (
                    ID INT AUTO_INCREMENT PRIMARY KEY,
                    Date DATETIME,
                    Object VARCHAR(50),
                    Service VARCHAR(50)
                )
            """)
            print("Table 'EventLog' created successfully.")

        except mysql.connector.Error as err:
            print("Failed creating table: {}".format(err))
            exit(1)

        finally:
            if self.cur:
                self.cur.close()
            if self.conn:
                self.conn.close()

    def insert_script(self, name, script):
        self.connect_database()  # 데이터베이스 연결
        try:
            self.cur.execute("INSERT INTO Scripts (name, script) VALUES (%s, %s)", (name, script))
            self.conn.commit()
            print("Script inserted successfully.")
        except mysql.connector.Error as err:
            print("Failed inserting script: {}".format(err))
            exit(1)
        finally:
            if self.cur:
                self.cur.close()
            if self.conn:
                self.conn.close()

if __name__ == "__main__":
    # DatabaseCreate 인스턴스 생성
    db_manager = DatabaseCreate(host="localhost", user="root")

    # 데이터베이스 생성 및 연결
    db_manager.create_database('ros_final')

    # 테이블 생성
    db_manager.create_table()

    # 데이터 삽입
    db_manager.insert_script('피라미드', '피라미드는 고대 이집트의 거대한 무덤입니다.')
    db_manager.insert_script('나폴레옹', '나폴레옹은 19세기 프랑스 황제로, 유럽을 정복한 군사 지도자입니다.')
    db_manager.insert_script('스핑크스', '스핑크스는 고대 이집트의 사자 몸에 사람 머리 석상입니다.')
    db_manager.insert_script('오벨리스크', '오벨리스크는 고대 이집트의 뾰족한 돌기둥 기념물입니다.')
    db_manager.insert_script('람세스', '람세스는 고대 이집트의 파라오입니다.')
    db_manager.insert_script('오시리스', '오시리스는 고대 이집트 신화의 지하의 왕입니다.')
