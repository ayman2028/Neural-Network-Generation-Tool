#!/usr/bin/env python
"""
Low-level Redis interaction script for experimentation.
This script connects directly to Redis (no Django) for direct command testing.

Usage:
    cd backend
    pipenv run python Experiments/redis_direct.py
    
Then use the 'r' object to interact with Redis:
    r.set('key', 'value')
    r.get('key')
    r.keys('*')
    etc.
"""

import redis
import json
from typing import Any, Dict, List

# Connect to Redis directly
r = redis.Redis(host='127.0.0.1', port=6379, db=0, decode_responses=True)

def print_header(text):
    print(f"\n{'='*60}")
    print(f"  {text}")
    print(f"{'='*60}\n")

def test_connection():
    """Test if Redis is connected"""
    try:
        result = r.ping()
        print(f"✓ Connected to Redis: {result}")
        return True
    except Exception as e:
        print(f"✗ Failed to connect: {e}")
        return False

def demo_strings():
    """String operations"""
    print_header("STRINGS")
    
    # SET and GET
    r.set('greeting', 'Hello Redis!')
    print(f"SET greeting='Hello Redis!'")
    print(f"GET greeting: {r.get('greeting')}")
    
    # APPEND
    r.append('greeting', ' Welcome!')
    print(f"APPEND ' Welcome!': {r.get('greeting')}")
    
    # STRLEN
    print(f"STRLEN: {r.strlen('greeting')}")
    
    # GETRANGE
    print(f"GETRANGE(0, 4): {r.getrange('greeting', 0, 4)}")
    
    # INCR/DECR
    r.set('counter', '0')
    r.incr('counter')
    r.incr('counter')
    print(f"INCR counter twice: {r.get('counter')}")
    r.decr('counter')
    print(f"DECR counter once: {r.get('counter')}")
    
    # Cleanup
    r.delete('greeting', 'counter')

def demo_hashes():
    """Hash (dict-like) operations"""
    print_header("HASHES")
    
    # HSET and HGET
    r.hset('user:1', mapping={
        'name': 'Alice',
        'email': 'alice@example.com',
        'age': '30'
    })
    print(f"HSET user:1: name, email, age")
    print(f"HGET user:1 name: {r.hget('user:1', 'name')}")
    
    # HGETALL
    user = r.hgetall('user:1')
    print(f"HGETALL user:1: {user}")
    
    # HINCRBY
    r.hincrby('user:1', 'age', 1)
    print(f"HINCRBY age by 1: {r.hget('user:1', 'age')}")
    
    # HKEYS, HVALS, HLEN
    print(f"HKEYS: {r.hkeys('user:1')}")
    print(f"HVALS: {r.hvals('user:1')}")
    print(f"HLEN: {r.hlen('user:1')}")
    
    # HEXISTS
    print(f"HEXISTS name: {r.hexists('user:1', 'name')}")
    print(f"HEXISTS phone: {r.hexists('user:1', 'phone')}")
    
    # Cleanup
    r.delete('user:1')

def demo_lists():
    """List operations (stacks/queues)"""
    print_header("LISTS")
    
    # LPUSH and RPUSH
    r.delete('tasks')  # Clear first
    r.lpush('tasks', 'task1', 'task2', 'task3')
    print(f"LPUSH tasks: task1, task2, task3")
    
    # LRANGE
    tasks = r.lrange('tasks', 0, -1)
    print(f"LRANGE 0, -1: {tasks}")
    
    # LPOP and RPOP
    left = r.lpop('tasks')
    right = r.rpop('tasks')
    print(f"LPOP: {left}, RPOP: {right}")
    print(f"Remaining: {r.lrange('tasks', 0, -1)}")
    
    # LLEN
    print(f"LLEN: {r.llen('tasks')}")
    
    # LINDEX
    r.rpush('tasks', 'task4', 'task5')
    print(f"LINDEX 0: {r.lindex('tasks', 0)}")
    print(f"LINDEX 1: {r.lindex('tasks', 1)}")
    
    # LINSERT
    r.linsert('tasks', 'BEFORE', 'task4', 'task3.5')
    print(f"LINSERT before task4: {r.lrange('tasks', 0, -1)}")
    
    # Cleanup
    r.delete('tasks')

def demo_sets():
    """Set operations (unique values)"""
    print_header("SETS")
    
    # SADD
    r.sadd('colors', 'red', 'green', 'blue')
    print(f"SADD colors: red, green, blue")
    
    # SMEMBERS
    colors = r.smembers('colors')
    print(f"SMEMBERS: {colors}")
    
    # SISMEMBER
    print(f"SISMEMBER red: {r.sismember('colors', 'red')}")
    print(f"SISMEMBER yellow: {r.sismember('colors', 'yellow')}")
    
    # SCARD
    print(f"SCARD: {r.scard('colors')}")
    
    # Set operations
    r.sadd('primary', 'red', 'green', 'blue')
    r.sadd('secondary', 'green', 'blue', 'yellow')
    
    print(f"\nSet theory:")
    print(f"SINTER (both sets): {r.sinter('primary', 'secondary')}")
    print(f"SUNION (either set): {r.sunion('primary', 'secondary')}")
    print(f"SDIFF (primary only): {r.sdiff('primary', 'secondary')}")
    
    # Cleanup
    r.delete('colors', 'primary', 'secondary')

def demo_sorted_sets():
    """Sorted Set operations (scored values)"""
    print_header("SORTED SETS")
    
    # ZADD
    r.zadd('leaderboard', {
        'alice': 100,
        'bob': 85,
        'charlie': 95,
        'diana': 110
    })
    print(f"ZADD leaderboard with scores")
    
    # ZRANGE (lowest to highest scores)
    print(f"ZRANGE 0, -1: {r.zrange('leaderboard', 0, -1)}")
    print(f"ZRANGE with WITHSCORES: {r.zrange('leaderboard', 0, -1, withscores=True)}")
    
    # ZREVRANGE (highest to lowest scores)
    print(f"ZREVRANGE 0, -1: {r.zrevrange('leaderboard', 0, -1, withscores=True)}")
    
    # ZCARD, ZCOUNT
    print(f"ZCARD: {r.zcard('leaderboard')}")
    print(f"ZCOUNT 80, 100: {r.zcount('leaderboard', 80, 100)}")
    
    # ZSCORE
    print(f"ZSCORE alice: {r.zscore('leaderboard', 'alice')}")
    
    # ZRANK
    print(f"ZRANK alice: {r.zrank('leaderboard', 'alice')}")
    print(f"ZREVRANK alice: {r.zrevrank('leaderboard', 'alice')}")
    
    # ZINCRBY
    r.zincrby('leaderboard', 5, 'bob')
    print(f"ZINCRBY bob by 5: {r.zscore('leaderboard', 'bob')}")
    
    # Cleanup
    r.delete('leaderboard')

def demo_expiration():
    """Key expiration"""
    print_header("EXPIRATION")
    
    # SET with EX
    r.set('temp_key', 'will_expire', ex=5)
    print(f"SET temp_key with 5 second expiration")
    print(f"GET temp_key: {r.get('temp_key')}")
    
    # TTL
    ttl = r.ttl('temp_key')
    print(f"TTL: {ttl} seconds")
    
    # PTTL (milliseconds)
    pttl = r.pttl('temp_key')
    print(f"PTTL: {pttl} milliseconds")
    
    # EXPIRE
    r.set('another_key', 'value')
    r.expire('another_key', 10)
    print(f"EXPIRE another_key to 10 seconds: TTL={r.ttl('another_key')}")
    
    # PERSIST
    r.persist('another_key')
    print(f"PERSIST another_key: TTL={r.ttl('another_key')}")
    
    # Cleanup
    r.delete('temp_key', 'another_key')

def demo_transactions():
    """Transaction/pipeline operations"""
    print_header("TRANSACTIONS & PIPELINES")
    
    # PIPELINE - send multiple commands at once
    pipe = r.pipeline()
    pipe.set('key1', 'value1')
    pipe.set('key2', 'value2')
    pipe.set('key3', 'value3')
    pipe.get('key1')
    pipe.get('key2')
    results = pipe.execute()
    print(f"Pipeline execute: {results}")
    
    # WATCH/MULTI/EXEC with proper transaction handling
    r.set('balance', '100')
    
    # Use watch and transaction properly
    with r.pipeline(transaction=True) as pipe:
        while True:
            try:
                pipe.watch('balance')
                balance = int(r.get('balance'))
                
                if balance >= 20:
                    pipe.multi()
                    pipe.decrby('balance', 20)
                    result = pipe.execute()
                    print(f"Transaction (deduct 20): Success, New balance: {r.get('balance')}")
                else:
                    pipe.unwatch()
                    print(f"Transaction (deduct 20): Failed - insufficient balance")
                break
            except redis.WatchError:
                continue
    
    # Cleanup
    r.delete('key1', 'key2', 'key3', 'balance')

def demo_scripting():
    """Lua scripting"""
    print_header("LUA SCRIPTING")
    
    # Simple script to increment with check
    script = r.register_script("""
        local val = redis.call('get', KEYS[1])
        if val then
            val = tonumber(val) + ARGV[1]
        else
            val = ARGV[1]
        end
        redis.call('set', KEYS[1], val)
        return val
    """)
    
    result = script(keys=['counter'], args=[5])
    print(f"Lua script result: {result}")
    print(f"Counter value: {r.get('counter')}")
    
    # Cleanup
    r.delete('counter')

def demo_key_operations():
    """Key management"""
    print_header("KEY OPERATIONS")
    
    # Set some keys
    r.set('user:1:name', 'Alice')
    r.set('user:1:email', 'alice@example.com')
    r.set('user:2:name', 'Bob')
    r.set('config:timeout', '30')
    
    # KEYS pattern matching
    print(f"KEYS user:*: {r.keys('user:*')}")
    print(f"KEYS user:1:*: {r.keys('user:1:*')}")
    print(f"KEYS *:name: {r.keys('*:name')}")
    
    # EXISTS
    print(f"EXISTS user:1:name: {r.exists('user:1:name')}")
    print(f"EXISTS user:3:name: {r.exists('user:3:name')}")
    
    # TYPE
    print(f"TYPE user:1:name: {r.type('user:1:name')}")
    
    # RENAME
    r.rename('config:timeout', 'config:timeout_seconds')
    print(f"RENAME config:timeout -> config:timeout_seconds")
    
    # DBSIZE
    print(f"DBSIZE (total keys): {r.dbsize()}")
    
    # Cleanup
    r.flushdb()
    print(f"\nFLUSHDB - cleared all keys")

def interactive_mode():
    """Interactive Redis shell"""
    print_header("INTERACTIVE MODE")
    print("You can now type Redis commands directly!")
    print("Examples:")
    print("  r.set('key', 'value')")
    print("  r.get('key')")
    print("  r.hset('hash', 'field', 'value')")
    print("  r.lpush('list', 'item')")
    print("  r.sadd('set', 'member')")
    print("  r.keys('*')")
    print("  r.flushdb()  # Warning: clears everything!")
    print("  exit()  # Exit this script")
    print(f"\nRedis connection object available as 'r'")
    print("="*60 + "\n")

# Run all demos
if __name__ == '__main__':
    if not test_connection():
        exit(1)
    
    print_header("REDIS LOW-LEVEL EXPERIMENTATION")
    
    # Run all demos
    demo_strings()
    demo_hashes()
    demo_lists()
    demo_sets()
    demo_sorted_sets()
    demo_expiration()
    demo_transactions()
    demo_scripting()
    demo_key_operations()
    
    # Start interactive mode
    interactive_mode()
    
    # Drop into Python interactive shell
    import code
    code.interact(local=locals(), banner="")
