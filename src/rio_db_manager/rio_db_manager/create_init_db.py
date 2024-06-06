class CreateInitDB():
    def __init__(self, db_manager):
        self.db_manager = db_manager

    def create_database(self):
        create_db_query = "CREATE DATABASE IF NOT EXISTS RiO_DB"
        use_db = "USE RiO_DB"
        self.db_manager.execute_query(create_db_query)
        self.db_manager.execute_query(use_db)

    def create_tables(self):
        tables = {
            "UserInfo": """
                user_id INT AUTO_INCREMENT PRIMARY KEY,
                office VARCHAR(16) NOT NULL,
                user_name VARCHAR(16) NOT NULL,
                birth DATE NOT NULL,
                phone_number VARCHAR(16) NOT NULL,
                user_face BLOB NOT NULL
            """,
            "Payment": """
                payment_id INT AUTO_INCREMENT PRIMARY KEY,
                date DATETIME NOT NULL,
                user_id INT NOT NULL,
                rfid_UID BIGINT NOT NULL,
                change_info VARCHAR(32) NOT NULL,
                change_credit INT NOT NULL,
                total_credit INT NOT NULL
            """,
            "RobotStatus": """
                status_id INT AUTO_INCREMENT PRIMARY KEY,
                date DATETIME NOT NULL,
                robot_id INT NOT NULL,
                status_info VARCHAR(128) NOT NULL,
                location_x INT NOT NULL,
                location_y INT NOT NULL
            """,            
            "RobotTask": """
                task_id INT AUTO_INCREMENT PRIMARY KEY,
                date DATETIME NOT NULL,
                robot_id INT NOT NULL,
                task_type VARCHAR(16) NOT NULL,
                status VARCHAR(16) NOT NULL
            """,
            "UsersRequest": """
                request_id INT AUTO_INCREMENT PRIMARY KEY,
                date DATETIME NOT NULL,
                user_id INT NOT NULL,
                service_type VARCHAR(16) NOT NULL,
                service_progress VARCHAR(16) NOT NULL,
                service_satisfaction INT NOT NULL
            """,
            "VisitorInfo": """
                visitor_id INT AUTO_INCREMENT PRIMARY KEY,
                visit_place VARCHAR(16) NOT NULL,
                purpose VARCHAR(64) NOT NULL,
                name VARCHAR(16) NOT NULL,
                phone_number VARCHAR(16) NOT NULL,
                affiliation VARCHAR(32) NOT NULL,
                visit_datetime DATETIME NOT NULL,
                robot_guidance VARCHAR(8) NOT NULL, 
                status VARCHAR(16) NOT NULL,
                registered_at DATETIME NOT NULL,
                updated_at DATETIME NOT NULL,
                hashed_data VARCHAR(64) NOT NULL
            """
        }

        for table_name, table_definition in tables.items():
            self.create_table(table_name, table_definition)
        

    def create_table(self, table_name, table_definition):
        query = f"CREATE TABLE IF NOT EXISTS {table_name} ({table_definition});"
        self.db_manager.execute_query(query)
