import mysql.connector
from datetime import datetime, timedelta

class DatabaseManager:
    def __init__(self, host, user):
        self.host = host
        self.user = "root"
        self.db_name = "ros_final"
        self.cur = None
        self.conn = None
        self.password = "guswns123"


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
            
            conditions = []
            parameters = []
            
            if service_name != 'ALL':
                conditions.append("Service = %s")
                parameters.append(service_name)
            
            if object_name != 'ALL':
                conditions.append("Object = %s")
                parameters.append(object_name)
            
            if start_date and end_date:
                # 종료일(end_date)에 하루를 추가하여 포함되도록 설정
                adjusted_end_date = (datetime.strptime(end_date, "%Y-%m-%d") + timedelta(days=1)).strftime("%Y-%m-%d")
                conditions.append("Date BETWEEN %s AND %s")
                parameters.extend([start_date, adjusted_end_date])
            
            # Construct the query based on the conditions
            if not conditions:
                query = "SELECT * FROM EventLog"
            else:
                query = "SELECT * FROM EventLog WHERE " + " AND ".join(conditions)
            
            # Debugging print statements
            print(f"Query: {query}")
            print(f"Parameters: {parameters}")
            
            self.cur.execute(query, tuple(parameters))
            return self.cur.fetchall()