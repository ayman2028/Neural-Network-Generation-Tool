from django.test import TestCase
from .models import NetworkConfig


class NetworkConfigTestCase(TestCase):
    """Test suite for NetworkConfig model"""
    
    def setUp(self):
        """Create test instances"""
        pass
    
    # ========== Mode 1 Tests ==========
    def test_create_mode1_network(self):
        """Test creating a Mode 1 (single unparallelized layer) network"""
        network = NetworkConfig.objects.create(
            name="Test Mode 1",
            description="Single layer network",
            mode=1,
            input_size=8,
            output_size=10,
            T=16,
            R=False,
            P=1
        )
        self.assertEqual(network.name, "Test Mode 1")
        self.assertEqual(network.mode, 1)
        self.assertEqual(network.input_size, 8)
        self.assertEqual(network.output_size, 10)
        self.assertEqual(network.T, 16)
        self.assertFalse(network.R)
        self.assertEqual(network.P, 1)
        self.assertEqual(network.layer_sizes, [])
        self.assertIsNone(network.B)
    
    def test_mode1_string_representation(self):
        """Test __str__ method for Mode 1 network"""
        network = NetworkConfig.objects.create(
            name="FC Layer",
            mode=1,
            input_size=8,
            output_size=10,
            T=16,
            R=False
        )
        self.assertEqual(str(network), "FC Layer (Mode 1)")
    
    # ========== Mode 2 Tests ==========
    def test_create_mode2_network(self):
        """Test creating a Mode 2 (parallelized layer) network"""
        network = NetworkConfig.objects.create(
            name="Test Mode 2",
            description="Parallelized layer network",
            mode=2,
            input_size=8,
            output_size=16,
            T=16,
            R=True,
            P=4
        )
        self.assertEqual(network.mode, 2)
        self.assertEqual(network.input_size, 8)
        self.assertEqual(network.output_size, 16)
        self.assertEqual(network.P, 4)
        self.assertTrue(network.R)
    
    def test_mode2_parallelism_factor(self):
        """Test that parallelism factor is stored correctly"""
        network = NetworkConfig.objects.create(
            name="Parallel Test",
            mode=2,
            input_size=10,
            output_size=20,
            T=16,
            R=False,
            P=5
        )
        self.assertEqual(network.P, 5)
    
    # ========== Mode 3 Tests ==========
    def test_create_mode3_network(self):
        """Test creating a Mode 3 (three-layer) network"""
        network = NetworkConfig.objects.create(
            name="Test Mode 3",
            description="Three-layer network",
            mode=3,
            input_size=4,
            T=16,
            R=False,
            layer_sizes=[8, 12, 16],
            B=10
        )
        self.assertEqual(network.mode, 3)
        self.assertEqual(network.input_size, 4)
        self.assertEqual(network.layer_sizes, [8, 12, 16])
        self.assertEqual(network.B, 10)
        self.assertIsNone(network.output_size)
    
    def test_mode3_different_layer_sizes(self):
        """Test Mode 3 with different layer configurations"""
        layers = [6, 12, 20]
        network = NetworkConfig.objects.create(
            name="Different Layers",
            mode=3,
            input_size=6,
            T=16,
            R=True,
            layer_sizes=layers,
            B=50
        )
        self.assertEqual(network.layer_sizes, layers)
        self.assertTrue(network.R)
        self.assertEqual(network.B, 50)
    
    # ========== Field Tests ==========
    def test_relu_activation_default_false(self):
        """Test that ReLU activation defaults to False"""
        network = NetworkConfig.objects.create(
            name="Default ReLU Test",
            mode=1,
            input_size=8,
            output_size=10,
            T=16
        )
        self.assertFalse(network.R)
    
    def test_parallelism_default_one(self):
        """Test that parallelism factor defaults to 1"""
        network = NetworkConfig.objects.create(
            name="Default P Test",
            mode=1,
            input_size=8,
            output_size=10,
            T=16
        )
        self.assertEqual(network.P, 1)
    
    def test_layer_sizes_default_empty_list(self):
        """Test that layer_sizes defaults to empty list"""
        network = NetworkConfig.objects.create(
            name="Default Layers Test",
            mode=1,
            input_size=8,
            output_size=10,
            T=16
        )
        self.assertEqual(network.layer_sizes, [])
    
    def test_description_optional(self):
        """Test that description is optional"""
        network = NetworkConfig.objects.create(
            name="No Description",
            mode=1,
            input_size=8,
            output_size=10,
            T=16
        )
        self.assertEqual(network.description, "")
    
    def test_generated_files_path_optional(self):
        """Test that generated_files_path is optional"""
        network = NetworkConfig.objects.create(
            name="No Path Test",
            mode=1,
            input_size=8,
            output_size=10,
            T=16
        )
        self.assertEqual(network.generated_files_path, "")
    
    # ========== Bit Width Tests ==========
    def test_various_bit_widths(self):
        """Test that different bit widths can be stored"""
        for bit_width in [8, 16, 32, 64]:
            network = NetworkConfig.objects.create(
                name=f"T={bit_width} Test",
                mode=1,
                input_size=8,
                output_size=10,
                T=bit_width
            )
            self.assertEqual(network.T, bit_width)
    
    # ========== Mode Choices Tests ==========
    def test_mode_choices(self):
        """Test that only valid modes can be created"""
        for mode in [1, 2, 3]:
            network = NetworkConfig.objects.create(
                name=f"Mode {mode} Test",
                mode=mode,
                input_size=8,
                output_size=10,
                T=16
            )
            self.assertEqual(network.mode, mode)
    
    # ========== Querying Tests ==========
    def test_retrieve_network_by_mode(self):
        """Test querying networks by mode"""
        NetworkConfig.objects.create(name="Mode1", mode=1, input_size=8, output_size=10, T=16)
        NetworkConfig.objects.create(name="Mode2", mode=2, input_size=8, output_size=16, T=16)
        NetworkConfig.objects.create(name="Mode3", mode=3, input_size=4, T=16, layer_sizes=[8, 12, 16])
        
        mode1_networks = NetworkConfig.objects.filter(mode=1)
        mode2_networks = NetworkConfig.objects.filter(mode=2)
        mode3_networks = NetworkConfig.objects.filter(mode=3)
        
        self.assertEqual(mode1_networks.count(), 1)
        self.assertEqual(mode2_networks.count(), 1)
        self.assertEqual(mode3_networks.count(), 1)
    
    def test_retrieve_network_by_name(self):
        """Test querying networks by name"""
        network = NetworkConfig.objects.create(
            name="Unique Network",
            mode=1,
            input_size=8,
            output_size=10,
            T=16
        )
        found = NetworkConfig.objects.get(name="Unique Network")
        self.assertEqual(found.id, network.id)
    
    # ========== Ordering Tests ==========
    def test_networks_ordered_by_created_date(self):
        """Test that networks are ordered by creation date (newest first)"""
        net1 = NetworkConfig.objects.create(name="First", mode=1, input_size=8, output_size=10, T=16)
        net2 = NetworkConfig.objects.create(name="Second", mode=1, input_size=8, output_size=10, T=16)
        net3 = NetworkConfig.objects.create(name="Third", mode=1, input_size=8, output_size=10, T=16)
        
        networks = NetworkConfig.objects.all()
        self.assertEqual(networks[0].name, "Third")  # Most recent first
        self.assertEqual(networks[1].name, "Second")
        self.assertEqual(networks[2].name, "First")
    
    # ========== Timestamp Tests ==========
    def test_created_at_timestamp(self):
        """Test that created_at is automatically set"""
        network = NetworkConfig.objects.create(
            name="Timestamp Test",
            mode=1,
            input_size=8,
            output_size=10,
            T=16
        )
        self.assertIsNotNone(network.created_at)
    
    def test_updated_at_timestamp(self):
        """Test that updated_at is automatically set"""
        network = NetworkConfig.objects.create(
            name="Update Timestamp Test",
            mode=1,
            input_size=8,
            output_size=10,
            T=16
        )
        self.assertIsNotNone(network.updated_at)
    
    # ========== Edge Cases ==========
    def test_large_input_size(self):
        """Test that large input sizes can be stored"""
        network = NetworkConfig.objects.create(
            name="Large Input",
            mode=1,
            input_size=1024,
            output_size=512,
            T=16
        )
        self.assertEqual(network.input_size, 1024)
    
    def test_large_output_size(self):
        """Test that large output sizes can be stored"""
        network = NetworkConfig.objects.create(
            name="Large Output",
            mode=1,
            input_size=8,
            output_size=2048,
            T=16
        )
        self.assertEqual(network.output_size, 2048)
    
    def test_mode3_with_many_layers(self):
        """Test Mode 3 configuration with extended layer support"""
        layers = [8, 12, 16, 20, 24]  # Future-proofing for >3 layers
        network = NetworkConfig.objects.create(
            name="Extended Layers",
            mode=3,
            input_size=4,
            T=16,
            R=False,
            layer_sizes=layers,
            B=100
        )
        self.assertEqual(network.layer_sizes, layers)
        self.assertEqual(len(network.layer_sizes), 5)
