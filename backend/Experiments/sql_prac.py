import psycopg2
from psycopg2 import sql
import os
import random
import string
import hashlib

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

def print_pract():
    conn = psycopg2.connect(**DB_CONFIG)
    cursor = conn.cursor()

    cursor.execute("SELECT * FROM test_users;")
    rows = cursor.fetchall()
    for row in rows:
        print(row)

    cursor.execute("SELECT * FROM post_table;")
    rows = cursor.fetchall()
    for row in rows:
        print(row)

    cursor.close()
    conn.close()


def generate_random_password(length=12):
    """Generate a random password"""
    characters = string.ascii_letters + string.digits + string.punctuation
    return ''.join(random.choice(characters) for _ in range(length))


def fill_test_users(num_users=10):
    """Fill test_users table with random data"""
    # List of random names
    first_names = ['Alice', 'Bob', 'Charlie', 'David', 'Emma', 'Frank', 
                   'Grace', 'Henry', 'Iris', 'Jack', 'Karen', 'Leo', 
                   'Mia', 'Noah', 'Olivia', 'Peter', 'Quinn', 'Rachel',
                   'Sam', 'Tina', 'Uma', 'Victor', 'Wendy', 'Xavier']
    
    last_names = ['Smith', 'Johnson', 'Williams', 'Brown', 'Jones', 'Garcia',
                  'Miller', 'Davis', 'Rodriguez', 'Martinez', 'Taylor', 'Anderson',
                  'Thomas', 'Moore', 'Jackson', 'Martin', 'Lee', 'Wilson']
    
    conn = psycopg2.connect(**DB_CONFIG)
    cursor = conn.cursor()
    
    inserted = 0
    skipped = 0
    
    for _ in range(num_users):
        name = f"{random.choice(first_names)} {random.choice(last_names)}"
        email = f"{name.lower().replace(' ', '.')}.{random.randint(1000, 9999)}@email.com"
        password = generate_random_password()
        is_active = random.choice([True, False])
        
        try:
            insert_query = """
            INSERT INTO test_users (email, password, is_active, name)
            VALUES (%s, %s, %s, %s);
            """
            cursor.execute(insert_query, (email, password, is_active, name))
            conn.commit()
            inserted += 1
            print(f"✓ Inserted: {name} ({email})")
            
        except psycopg2.IntegrityError as error:
            # Email already exists (UNIQUE constraint)
            conn.rollback()
            skipped += 1
            print(f"✗ Skipped: {email} (already exists)")
    
    cursor.close()
    conn.close()
    
    print(f"\n✓ Inserted: {inserted}, ✗ Skipped: {skipped}")


def fill_post_table(num_posts=20):
    """Fill post_table with random test data"""
    # Sample post titles and content
    titles = [
        "Getting Started with Python",
        "Understanding SQL Joins",
        "Best Practices in Web Development",
        "Introduction to PostgreSQL",
        "Database Design Tips",
        "Advanced Python Techniques",
        "Building Scalable Applications",
        "REST API Development",
        "Django Framework Deep Dive",
        "Data Structures Explained",
        "Performance Optimization",
        "Cloud Computing Basics",
        "Machine Learning Fundamentals",
        "Security Best Practices",
        "Testing Your Code"
    ]
    
    content_samples = [
        "This is a comprehensive guide to get you started with this topic.",
        "Learn the essential concepts and best practices.",
        "Discover tips and tricks from experienced developers.",
        "A detailed walkthrough of the key concepts.",
        "Understanding the fundamentals will help you master this skill.",
        "Practical examples and real-world applications.",
        "Common mistakes to avoid when implementing this.",
        "Advanced techniques for improving performance."
    ]
    
    conn = psycopg2.connect(**DB_CONFIG)
    cursor = conn.cursor()
    
    # Get all user IDs from test_users table
    cursor.execute("SELECT id FROM test_users;")
    user_ids = [row[0] for row in cursor.fetchall()]
    
    if not user_ids:
        print("No users found. Run fill_test_users() first.\n")
        cursor.close()
        conn.close()
        return
    
    inserted = 0
    skipped = 0
    
    for _ in range(num_posts):
        title = random.choice(titles)
        content = " ".join(random.choices(content_samples, k=random.randint(1, 3)))
        author_id = random.choice(user_ids)
        
        try:
            insert_query = """
            INSERT INTO post_table (title, content, author_id)
            VALUES (%s, %s, %s);
            """
            cursor.execute(insert_query, (title, content, author_id))
            conn.commit()
            inserted += 1
            print(f"✓ Inserted post: {title}")
            
        except psycopg2.DatabaseError as error:
            conn.rollback()
            skipped += 1
            print(f"✗ Skipped post: {error}")
    
    cursor.close()
    conn.close()
    
    print(f"\n✓ Inserted: {inserted}, ✗ Skipped: {skipped}")


def pract():
    conn = psycopg2.connect(**DB_CONFIG)
    cursor = conn.cursor()

    cursor.execute("DROP TABLE IF EXISTS test_users;")
    conn.commit()

    create_table_query = """
    CREATE TABLE IF NOT EXISTS test_users (
        id SERIAL PRIMARY KEY NOT NULL,
        email VARCHAR(100) UNIQUE NOT NULL,
        password VARCHAR(100) NOT NULL,
        is_active BOOLEAN DEFAULT TRUE,
        created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
        name VARCHAR(50)
    );
    """

    cursor.execute(create_table_query) 
    conn.commit()
    fill_test_users(num_users=15)

    cursor.execute("DROP TABLE IF EXISTS post_table;")
    conn.commit()
    
    create_table_query = """
    CREATE TABLE IF NOT EXISTS post_table (
        id SERIAL PRIMARY KEY NOT NULL,
        title VARCHAR(200) NOT NULL,
        content TEXT,
        author_id INTEGER REFERENCES test_users(id) ON DELETE CASCADE
    );
    """

    cursor.execute(create_table_query)
    conn.commit()
    print("Table 'post_table' created successfully.\n")
    
    """
    cursor.execute("SELECT id, email FROM test_users;")
    rows = cursor.fetchall()
    for row in rows:
        print(row)
    """
    cursor.close()
    conn.close()
    
    # Fill with random data
    #fill_test_users(num_users=15)
    fill_post_table(num_posts=10)




if __name__ == "__main__":
    # Initialize database config from environment
    init_db_config()
    #pract()
    #fill_test_users()
    #print("\n--- All Users ---")
    print_pract()
