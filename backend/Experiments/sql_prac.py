import psycopg2
from psycopg2 import sql
import os

# Database connection parameters
DB_CONFIG = {}

def init_db_config():
    """Initialize DB_CONFIG with Django settings from environment variables"""
    global DB_CONFIG
    
    # Get database parameters from environment (same as Django settings.py)
    host = os.environ.get('DB_HOST', 'localhost')
    port = os.environ.get('DB_PORT', '5432')
    database = os.environ.get('DB_NAME', 'neural_network')
    user = os.environ.get('DB_USER', 'postgres')
    password = os.environ.get('DB_PASSWORD', 'postgres')
    
    DB_CONFIG = {
        "host": host,
        "port": port,
        "database": database,
        "user": user,
        "password": password
    }
    
    print(f"✓ Database config initialized:")
    print(f"  Host: {host}:{port}")
    print(f"  Database: {database}")
    print(f"  User: {user}\n")
    
    return DB_CONFIG

def practice_queries():
    """Write your hardcoded SQL queries here"""
    try:
        conn = psycopg2.connect(**DB_CONFIG)
        cursor = conn.cursor()
        
        # Test 1: Check database version
        cursor.execute("SELECT version();")
        version = cursor.fetchone()
        print(f"PostgreSQL Version: {version[0]}\n")
        
        # Test 2: List all tables in the database
        cursor.execute("""
            SELECT tablename FROM pg_tables 
            WHERE schemaname = 'public'
            ORDER BY tablename;
        """)
        tables = cursor.fetchall()
        print("Tables in database:")
        if tables:
            for table in tables:
                print(f"  - {table[0]}")
        else:
            print("  (No tables yet - run Django migrations)")
        print()
        
        # Query 1: Get all users (if table exists)
        try:
            cursor.execute("SELECT * FROM accounts_user LIMIT 5;")
            users = cursor.fetchall()
            print("Users:")
            for user in users:
                print(user)
            print()
        except psycopg2.DatabaseError:
            print("accounts_user table doesn't exist yet\n")
        
        cursor.close()
        conn.close()
        
    except psycopg2.DatabaseError as error:
        print(f"Error: {error}")

def sql_shell():
    """Interactive SQL shell for practice"""
    try:
        conn = psycopg2.connect(**DB_CONFIG)
        cursor = conn.cursor()
        
        print("PostgreSQL Interactive Shell")
        print("Type SQL queries and press Enter to execute")
        print("Commands: 'exit' to quit, 'clear' to clear screen\n")
        
        # Test connection
        cursor.execute("SELECT version();")
        print(f"Connected: {cursor.fetchone()[0]}\n")
        
        while True:
            try:
                # Get user input
                query = input("SQL> ").strip()
                
                # Handle special commands
                if query.lower() == "exit":
                    print("Closing connection...")
                    break
                elif query.lower() == "clear":
                    print("\033[2J\033[H", end="")  # Clear screen
                    continue
                elif not query:
                    continue
                
                # Execute query
                cursor.execute(query)
                conn.commit()
                
                # Fetch and display results
                if cursor.description:  # SELECT query with results
                    results = cursor.fetchall()
                    if results:
                        # Print column names
                        cols = [desc[0] for desc in cursor.description]
                        print("\t".join(cols))
                        print("-" * 80)
                        # Print rows
                        for row in results:
                            print("\t".join(str(val) for val in row))
                        print(f"\n({len(results)} rows)\n")
                    else:
                        print("(No results)\n")
                else:  # INSERT, UPDATE, DELETE
                    print(f"Rows affected: {cursor.rowcount}\n")
                    
            except psycopg2.DatabaseError as error:
                print(f"Error: {error}\n")
                conn.rollback()
        
        cursor.close()
        conn.close()
        
    except psycopg2.DatabaseError as error:
        print(f"Connection error: {error}")

if __name__ == "__main__":
    # Initialize database config from environment
    init_db_config()
    
    # Run interactive SQL shell
    sql_shell()
