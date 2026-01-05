from django.test import TestCase
from django.contrib.auth import get_user_model
from networks.models import NetworkConfig

User = get_user_model()


class CustomUserTestCase(TestCase):
    """Test suite for CustomUser model"""
    
    def setUp(self):
        """Create test user instances"""
        self.user1 = User.objects.create_user(
            username='testuser1',
            email='test1@example.com',
            password='testpass123'
        )
        self.user2 = User.objects.create_user(
            username='testuser2',
            email='test2@example.com',
            password='testpass123'
        )
    
    def test_create_user(self):
        """Test creating a CustomUser"""
        self.assertEqual(self.user1.username, 'testuser1')
        self.assertEqual(self.user1.email, 'test1@example.com')
        self.assertTrue(self.user1.check_password('testpass123'))
    
    def test_email_unique(self):
        """Test that email field must be unique"""
        with self.assertRaises(Exception):
            User.objects.create_user(
                username='anotheruser',
                email='test1@example.com',  # Duplicate email
                password='testpass123'
            )
    
    def test_user_networks_reverse_relationship(self):
        """Test that users can access networks via reverse relationship"""
        network1 = NetworkConfig.objects.create(
            name="User1 Network 1",
            mode=1,
            input_size=8,
            output_size=10,
            T=16,
            R=False,
            P=1
        )
        network2 = NetworkConfig.objects.create(
            name="User1 Network 2",
            mode=2,
            input_size=6,
            output_size=8,
            T=16,
            R=False,
            P=2
        )
        
        # Add networks to user1
        network1.users.add(self.user1)
        network2.users.add(self.user1)
        
        # Verify user can access networks
        self.assertEqual(self.user1.networks.count(), 2)
        self.assertIn(network1, self.user1.networks.all())
        self.assertIn(network2, self.user1.networks.all())
    
    def test_user_has_multiple_networks(self):
        """Test that a user can be associated with multiple networks"""
        networks = []
        for i in range(3):
            network = NetworkConfig.objects.create(
                name=f"Multi Network {i}",
                mode=1,
                input_size=8 + i,
                T=16,
                R=False
            )
            network.users.add(self.user1)
            networks.append(network)
        
        self.assertEqual(self.user1.networks.count(), 3)
    
    def test_network_shared_between_users(self):
        """Test that a network can be shared between multiple users"""
        shared_network = NetworkConfig.objects.create(
            name="Shared Network",
            mode=1,
            input_size=8,
            T=16,
            R=False
        )
        
        # Add both users to the same network
        shared_network.users.add(self.user1, self.user2)
        
        # Both users should see the network
        self.assertIn(shared_network, self.user1.networks.all())
        self.assertIn(shared_network, self.user2.networks.all())
        self.assertEqual(shared_network.users.count(), 2)
    
    def test_user_deletion_preserves_networks(self):
        """Test that deleting a user doesn't delete their networks"""
        network = NetworkConfig.objects.create(
            name="User Network",
            mode=1,
            input_size=8,
            T=16,
            R=False
        )
        network.users.add(self.user1)
        
        user1_id = self.user1.id
        self.user1.delete()
        
        # Network should still exist but user association should be removed
        network.refresh_from_db()
        self.assertEqual(network.users.count(), 0)
        self.assertFalse(User.objects.filter(id=user1_id).exists())

