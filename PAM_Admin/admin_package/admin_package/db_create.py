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
    db_manager.insert_script('아메리칸 슈렉', '"아메리칸 고딕"에서 농부와 그의 딸이 슈렉과 프린세스 피오나로 나타나는 것은 전형적인 동화 속 이야기와 현실의 만남을 재미있게 표현한 작품입니다. 농부는 현실적인 농업 생활에 바쁜데, 그의 딸은 동화 속 세계에 빠져들고 싶어 합니다. 이 작품은 우리의 일상과 상상의 경계를 넘나들며, 현실과 꿈 사이에서 우리가 발견해야 할 균형을 생각하게 합니다.')
    db_manager.insert_script('자가격리 중인 예수', '이 작품은 "최후의 만찬"을 모티브로 삼아, 예수님이 마스크를 착용하고 자가격리 중인 모습을 재미있게 표현한 것입니다. 원래 예수님과 제자들이 함께하는 장면을 보는 것이 일반적이지만, 이 작품에서는 예수님만이 혼자 있는데, 그의 마스크 착용은 현실 세계의 코로나 시대에 대한 풍자적인 반영입니다. 이런 비꼼적인 접근은 우리에게 웃음과 함께 생각할 거리를 제공합니다.')
    db_manager.insert_script('바트심슨의 절규', '이 작품은 에드워드 먼크의 유명한 그림 "절규"를 바탕으로, 바트 심슨의 특징적인 표정과 스타일을 결합하여 새로운 시각으로 재해석한 것입니다. "절규"의 원본에서는 인간의 절규와 고통이 표현되어 있지만, 여기서는 바트 심슨의 유쾌한 이미지가 그 대신에 들어가 있습니다. 이 작품은 웃음과 유머를 통해 고전적인 예술 작품을 새롭게 만들어냄으로써 우리의 시각을 확장시키고, 즐거움을 선사합니다.')
    db_manager.insert_script('스쿠터 탄 나폴레옹', '이 작품은 나폴레옹이 거대한 알프스 산맥을 넘어가려는 유명한 장면을 바탕으로 하되, 그의 고귀한 말을 대신해 스쿠터를 타고 있는 모습을 담고 있습니다. 나폴레옹의 히어로적인 모습이 아닌, 보다 유쾌하고 유머러스한 측면을 강조한 이 작품은 역사적인 순간을 새로운 시선으로 바라보게 합니다. 스쿠터를 타고 알프스를 넘어가는 나폴레옹의 모습은 현대적이고 재미있으며, 우리에게 역사적 인물을 새롭게 보게 합니다.')