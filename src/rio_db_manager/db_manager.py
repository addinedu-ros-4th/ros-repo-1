import pymysql

class DBManager():
    def __init__(self, db_info):
        self.db_info = db_info
        self.connection = None
        self.connect_db()

    def connect_db(self):
        try:
            self.connection = pymysql.connect(
                host=self.db_info['host'],
                port=self.db_info['port'],
                user=self.db_info['user'],
                password=self.db_info['password'],
                database=self.db_info['database'],
                cursorclass=pymysql.cursors.DictCursor
            )
        except pymysql.MySQLError as e:
            print(f"Error connecting to MySQL: {e}")
            raise
    
    def execute_query(self, query, values=None):
        try:
            with self.connection.cursor() as cursor:
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
        # print(result)  # 결과 확인
        if result:
            # 쿼리 결과에서 테이블 이름만 추출하여 반환
            return [row['Tables_in_{}'.format(self.db_info['database'])] for row in result]
        else:
            return []  # 빈 리스트 반환 또는 None 반환

    def create(self, table, data):
        columns = ', '.join(data.keys())
        placeholders = ', '.join(['%s'] * len(data))
        query = f"INSERT INTO {table} ({columns}) VALUES ({placeholders})"
        values = tuple(data.values())
        self.execute_query(query, values)

    def read(self, table, criteria=None):
        query = f"SELECT * FROM {table}"
        values = ()
        if criteria:
            where_clause = ' AND '.join([f"{key}=%s" for key in criteria.keys()])
            query += f" WHERE {where_clause}"
            values = tuple(criteria.values())
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