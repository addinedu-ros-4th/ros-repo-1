from rio_db_manager.db_manager import DBManager
from rio_db_manager.create_init_db import CreateInitDB
from rio_ui.TTS_service import TTSService

class DBConnector(): # 싱글톤 패턴으로 구현
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super(DBConnector, cls).__new__(cls, *args, **kwargs)
        return cls._instance

    def __init__(self):
        if not hasattr(self, 'initialized'):  # 이 코드가 init이 여러 번 호출되지 않도록 합니다.
            self.db_manager = None
            self.db_connect()
            self.initialized = True

    def db_connect(self):
        self.db_manager = DBManager("db_config.json")
        db_initializer = CreateInitDB(self.db_manager)
        db_initializer.create_database()
        db_initializer.create_tables()
        
        return self.db_manager
    
    def show_all_tables(self):
        tables = self.db_manager.show_tables()
        
        return tables

    def select_all(self, table):
        all_table_data = self.db_manager.read(table)

        return all_table_data
    
    def select_specific(self, table, column, criteria=None):
        specific_data = self.db_manager.select(column, table, criteria)
        
        return specific_data
    
    def select_specific_null(self, table, column, criteria=None):
        specific_data = self.db_manager.select_null(column, table, criteria)
        
        return specific_data
    
    def insert_value(self, table, value):
        insert_into_table = self.db_manager.create(table, value)
        
        return insert_into_table
    
    def update_value(self, table, data, criteria=None):
        update_data = self.db_manager.update(table, data, criteria)
        
        return update_data
    
class TTSAlertService():     
    def __init__(self):
        self.ttsservice = TTSService()
        self.base_path = '/home/subin/project_ws/ros-repo-1/src/rio_ui/rio_ui/data/tts_mp3_files'
        self.tts_thread = None
        
    def create_tts_speak(self, file_name, text):
        file_path = os.path.join(self.base_path, f"{file_name}.mp3")
        self.ttsservice.create_tts_file(file_path, text)
        self.ttsservice.speak(file_path)
    
    def tts_speak(self, text):
        file_name = f"{text}.mp3"
        file_path = os.path.join(self.base_path, file_name)
        self.ttsservice.speak(file_path)

    def run_create_tts(self, file_name, text):
        self.stop_tts()
        self.tts_thread = threading.Thread(target=self.create_tts_speak, args=(file_name, text,))
        self.tts_thread.start()

    def run_tts(self, text):
        self.stop_tts()
        self.tts_thread = threading.Thread(target=self.tts_speak, args=(text,))
        self.tts_thread.start()

    def stop_tts(self):
        if self.tts_thread and self.tts_thread.is_alive():
            self.tts_thread.join()
        self.tts_thread = None
    
def main():
    pass

if __name__ == "__main__":
    main()