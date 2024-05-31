import mysql.connector

class DatabaseCreate:
    def __init__(self, host, user):
        self.host = host
        self.user = user
        self.db_name = "ros_final"
        self.cur = None
        self.conn = None
        self.password = "1234"

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
    db_manager.insert_script('강아지', '충실하고 충성스러운 강아지는, 인간의 가장 충실한 동반자로서 우정과 충실함을 상징합니다.')
    db_manager.insert_script('고양이', '우아하고 독립적인 존재 고양이는, 그림자 속에서 조용히 미묘한 아름다움을 표현하며 신비로움과 자유를 대표합니다.')
    db_manager.insert_script('초록양', '초록 초장을 향해 움직이는 양은 자연의 조화와 평온을 상징하며, 우리가 잊고 있던 간결한 아름다움을 알려줍니다.')
    db_manager.insert_script('갈색말', '넓은 목초지를 누비며 힘차게 달리는 말은 자유와 전유의 상징이자 우아함과 힘의 화려한 조화를 상징합니다.')
