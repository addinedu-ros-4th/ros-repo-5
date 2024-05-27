import mysql.connector
import pandas as pd

class DatabaseManager:
    def __init__(self, host, user):
        self.host = host
        self.user = "root"
        self.db_name = "ros_final"
        self.cur = None
        self.conn = None
        self.password = "your_password"

    def create_database(self, db_name):
        try:
            self.cur.execute("CREATE DATABASE IF NOT EXISTS {}".format(db_name))
        except mysql.connector.Error as err:
            print("Failed creating database: {}".format(err))
            exit(1)
    
    def create_table(self):
        try:
            self.cur.execute("""
                CREATE TABLE IF NOT EXISTS Scripts (
                    name CHAR(255),
                    script TEXT
                )
            """)
        except mysql.connector.Error as err:
            print("Failed creating table: {}".format(err))
            exit(1)

    def insert_script(self, name, script):
        try:
            self.cur.execute("INSERT INTO Scripts (name, script) VALUES (%s, %s)", (name, script))
            self.conn.commit()
            print("Script inserted successfully.")
        except mysql.connector.Error as err:
            print("Failed inserting script: {}".format(err))
            exit(1)

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
                self.create_database(db_name)
                self.conn.database = db_name
            else:
                raise
        self.cur = self.conn.cursor()

    
    def find_script_by_name(self, name):
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
