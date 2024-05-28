import mysql.connector
from datetime import datetime

class DatabaseManager:
    def __init__(self, host, user):
        self.host = host
        self.user = "root"
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

        except mysql.connector.Error as err:
            if err.errno == mysql.connector.errorcode.ER_BAD_DB_ERROR:
                self.conn = mysql.connector.connect(
                    host=self.host,
                    user=self.user,
                    password=self.password
                )
                self.cur = self.conn.cursor()
                self.conn.database = db_name
            else:
                raise
        self.cur = self.conn.cursor()

    
    def find_script_by_name(self, name):
        self.connect_database() 
        try:
            query = "SELECT script FROM Scripts WHERE name = %s"
            self.cur.execute(query, (name,))
            script = self.cur.fetchone()
            if script:
                return script[0]
            else:
                return None
        except mysql.connector.Error as err:
            print("Error:", err)
            return None

    def insert_event_log(self, object_name, service_name):
        self.connect_database() 
        try:
            current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            query = "INSERT INTO EventLog (Date, Object, Service) VALUES (%s, %s, %s)"
            self.cur.execute(query, (current_time, object_name, service_name))
            self.conn.commit()
            print("Event log inserted successfully.")
        except mysql.connector.Error as err:
            print("Failed inserting event log:", err)

    def search_event_logs(self, service_name, object_name, start_date, end_date):
        self.connect_database() 
        query = """
        SELECT * FROM EventLog 
        WHERE Service = %s AND Object = %s AND Date BETWEEN %s AND %s
        """
        self.cur.execute(query, (service_name, object_name, start_date, end_date))
        return self.cur.fetchall()