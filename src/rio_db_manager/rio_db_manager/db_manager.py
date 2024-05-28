import pymysql
import yaml
import os

from ament_index_python.packages import get_package_share_directory


class DBManager():
    def __init__(self, config_file):
        self.load_config(config_file)
        self.connection = None
        self.connect_db()
        
    def load_config(self, config_file):
        with open(os.path.join(get_package_share_directory("rio_db_manager"), "config", config_file)) as f:
            self.config = yaml.safe_load(f)
            self.config = self.config["rio_database"]

    def connect_db(self):
        try:
            self.connection = pymysql.connect(
            host=self.config['host'],
            user=self.config['user'],
            password=self.config['password'],
            port=self.config['port'],
            database=self.config["database"]
        )
        except pymysql.MySQLError as e:
            print(f"Error connecting to MySQL: {e}")
            raise
    
    def execute_query(self, query, values=None):
        try:
            with self.connection.cursor(pymysql.cursors.DictCursor) as cursor:
                if values:
                    cursor.execute(query, values)
                else:
                    cursor.execute(query)
                self.connection.commit()
                return cursor.fetchall()
        except pymysql.MySQLError as e:
            print(f"Error executing MySQL query: {e}")
            raise

    def show_tables(self):
        query = "SHOW TABLES;"
        result = self.execute_query(query)
        if result:
            # 쿼리 결과에서 테이블 이름만 추출하여 반환
            return [row['Tables_in_{}'.format(self.config["database"])] for row in result]
        else:
            return []  # 빈 리스트 반환 또는 None 반환

    def create(self, table, data):
        columns = ', '.join(data.keys())
        placeholders = ', '.join(['%s'] * len(data))
        query = f"INSERT INTO {table} ({columns}) VALUES ({placeholders})"
        values = tuple(data.values())
        self.execute_query(query, values)

    def read(self, table, criteria=None, option=None):
        query = f"SELECT * FROM {table}"
        values = ()
        if criteria:
            where_clause = ' AND '.join([f"{key}=%s" for key in criteria.keys()])
            query += f" WHERE {where_clause}"
            values = tuple(criteria.values())
        if option:
            query += f" ORDER BY {option} ASC"    
        
        return self.execute_query(query, values)

    def update(self, table, data, criteria):
        set_clause = ', '.join([f"{key}=%s" for key in data.keys()])
        where_clause = ' AND '.join([f"{key}=%s" for key in criteria.keys()])
        query = f"UPDATE {table} SET {set_clause} WHERE {where_clause}"
        values = tuple(data.values()) + tuple(criteria.values())
        self.execute_query(query, values)

    def delete(self, table, criteria):
        where_clause = ' AND '.join([f"{key}=%s" for key in criteria.keys()])
        query = f"DELETE FROM {table} WHERE {where_clause}"
        values = tuple(criteria.values())
        self.execute_query(query, values)

    def close(self):
        if self.connection:
            self.connection.close()