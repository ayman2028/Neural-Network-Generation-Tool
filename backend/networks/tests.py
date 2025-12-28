from django.test import TestCase
from django.urls import reverse
from django.contrib.auth import get_user_model
from unittest.mock import patch, MagicMock
from .models import NetworkConfig
from .forms import NetworkConfigForm
from django_redis import get_redis_connection


User = get_user_model()


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


class NetworkConfigFormTestCase(TestCase):
    """Test suite for NetworkConfigForm"""
    
    def test_form_includes_b_field(self):
        """Test that the form includes the B (multiplier budget) field"""
        form = NetworkConfigForm()
        self.assertIn('B', form.fields)
    
    def test_form_includes_all_required_fields(self):
        """Test that form includes all required network parameters"""
        form = NetworkConfigForm()
        expected_fields = ['name', 'description', 'mode', 'input_size', 'T', 'R', 'B']
        for field in expected_fields:
            self.assertIn(field, form.fields)
    
    def test_b_field_has_default_value(self):
        """Test that B field has default value of 10 for new instances"""
        form = NetworkConfigForm()
        self.assertEqual(form.fields['B'].initial, 10)
    
    def test_form_valid_with_mode1(self):
        """Test form validation for Mode 1 configuration"""
        form_data = {
            'name': 'Test Mode 1',
            'description': 'Test description',
            'mode': 1,
            'input_size': 8,
            'T': 16,
            'R': False,
            'B': 10
        }
        form = NetworkConfigForm(data=form_data)
        self.assertTrue(form.is_valid())
    
    def test_form_valid_with_mode3(self):
        """Test form validation for Mode 3 configuration"""
        form_data = {
            'name': 'Test Mode 3',
            'description': 'Three-layer network',
            'mode': 3,
            'input_size': 4,
            'T': 16,
            'R': True,
            'B': 10
        }
        form = NetworkConfigForm(data=form_data)
        self.assertTrue(form.is_valid())
    
    def test_form_with_custom_b_value(self):
        """Test form accepts custom B values"""
        form_data = {
            'name': 'Custom B',
            'description': '',
            'mode': 3,
            'input_size': 4,
            'T': 16,
            'R': False,
            'B': 50
        }
        form = NetworkConfigForm(data=form_data)
        self.assertTrue(form.is_valid())
        self.assertEqual(form.cleaned_data['B'], 50)
    
    def test_form_missing_required_field(self):
        """Test form validation fails when required field is missing"""
        form_data = {
            'description': 'Missing name',
            'mode': 1,
            'input_size': 8,
            'T': 16,
            'R': False,
            'B': 10
        }
        form = NetworkConfigForm(data=form_data)
        self.assertFalse(form.is_valid())
        self.assertIn('name', form.errors)
    
    def test_form_help_text_for_b_field(self):
        """Test that B field has helpful help text"""
        form = NetworkConfigForm()
        b_field = form.fields['B']
        self.assertIn('Mode 3', b_field.help_text)


class NetworkCreateViewTestCase(TestCase):
    """Test suite for NetworkCreateView"""
    
    def setUp(self):
        """Set up test user and client"""
        self.user = User.objects.create_user(
            username='testuser',
            email='test@example.com',
            password='testpass123'
        )
        self.client.login(username='testuser', password='testpass123')
    
    def test_create_view_get_request(self):
        """Test GET request to create view displays form"""
        response = self.client.get(reverse('networks:create'))
        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'networks/network_form.html')
        self.assertIsInstance(response.context['form'], NetworkConfigForm)
    
    def test_create_view_post_valid_data(self):
        """Test POST with valid data creates network and saves to database"""
        form_data = {
            'name': 'New Network',
            'description': 'Test network creation',
            'mode': 1,
            'input_size': 8,
            'T': 16,
            'R': False,
            'B': 10
        }
        response = self.client.post(reverse('networks:create'), form_data)
        
        # Check that network was created
        self.assertEqual(NetworkConfig.objects.count(), 1)
        network = NetworkConfig.objects.first()
        self.assertEqual(network.name, 'New Network')
        self.assertEqual(network.mode, 1)
        self.assertEqual(network.B, 10)
    
    def test_create_view_post_redirects_to_detail(self):
        """Test that successful POST redirects to detail view"""
        form_data = {
            'name': 'Redirect Test',
            'description': 'Test redirect',
            'mode': 1,
            'input_size': 8,
            'T': 16,
            'R': False,
            'B': 10
        }
        response = self.client.post(reverse('networks:create'), form_data, follow=False)
        
        network = NetworkConfig.objects.first()
        expected_url = reverse('networks:detail', kwargs={'pk': network.pk})
        self.assertRedirects(response, expected_url)
    
    @patch('networks.views.generate_network')
    def test_create_view_calls_generate_network(self, mock_generate):
        """Test that create view calls the generator with network config"""
        # Mock the generator to return a ZIP file path
        mock_generate.return_value = '/path/to/network_1.zip'
        
        form_data = {
            'name': 'Generator Test',
            'description': 'Test generator call',
            'mode': 1,
            'input_size': 8,
            'T': 16,
            'R': False,
            'B': 10
        }
        self.client.post(reverse('networks:create'), form_data)
        
        # Verify generator was called
        self.assertTrue(mock_generate.called)
        called_network = mock_generate.call_args[0][0]
        self.assertEqual(called_network.name, 'Generator Test')
    
    @patch('networks.views.generate_network')
    def test_create_view_stores_generated_path(self, mock_generate):
        """Test that generated file path is stored in model"""
        # Mock the generator
        expected_path = '/path/to/generated/network_1.zip'
        mock_generate.return_value = expected_path
        
        form_data = {
            'name': 'Path Storage Test',
            'description': 'Test path storage',
            'mode': 1,
            'input_size': 8,
            'T': 16,
            'R': False,
            'B': 10
        }
        self.client.post(reverse('networks:create'), form_data)
        
        network = NetworkConfig.objects.first()
        self.assertEqual(network.generated_files_path, expected_path)
    
    @patch('networks.views.generate_network')
    def test_create_view_populates_mode1_parameters(self, mock_generate):
        """Test that Mode 1 network populates output_size and P after generation"""
        mock_generate.return_value = '/path/to/network_1.zip'
        
        form_data = {
            'name': 'Mode 1 Params Test',
            'description': 'Test Mode 1 parameter population',
            'mode': 1,
            'input_size': 8,
            'T': 16,
            'R': False,
            'B': 10
        }
        self.client.post(reverse('networks:create'), form_data)
        
        network = NetworkConfig.objects.first()
        # Mode 1: output_size should equal input_size
        self.assertEqual(network.output_size, 8)
        # Mode 1: P should always be 1 (unparallelized)
        self.assertEqual(network.P, 1)
        self.assertEqual(network.generated_files_path, '/path/to/network_1.zip')
    
    @patch('networks.views.generate_network')
    def test_create_view_populates_mode3_parameters(self, mock_generate):
        """Test that Mode 3 network populates layer_sizes after generation"""
        mock_generate.return_value = '/path/to/network_1.zip'
        
        form_data = {
            'name': 'Mode 3 Params Test',
            'description': 'Test Mode 3 parameter population',
            'mode': 3,
            'input_size': 4,
            'T': 16,
            'R': True,
            'B': 10
        }
        self.client.post(reverse('networks:create'), form_data)
        
        network = NetworkConfig.objects.first()
        # Mode 3: layer_sizes should be [M1, M2, M3] = [input_size, input_size, 1]
        self.assertEqual(network.layer_sizes, [4, 4, 1])
        self.assertEqual(network.generated_files_path, '/path/to/network_1.zip')
    
    @patch('networks.views.generate_network')
    def test_create_view_handles_generation_error(self, mock_generate):
        """Test that creation succeeds even if generation fails, and parameters NOT populated"""
        from generator.generator import GenerationError
        
        # Mock the generator to raise an error
        mock_generate.side_effect = GenerationError("Generation failed")
        
        form_data = {
            'name': 'Error Handling Test',
            'description': 'Test error handling',
            'mode': 1,
            'input_size': 8,
            'T': 16,
            'R': False,
            'B': 10
        }
        response = self.client.post(reverse('networks:create'), form_data)
        
        # Network should still be created
        self.assertEqual(NetworkConfig.objects.count(), 1)
        network = NetworkConfig.objects.first()
        self.assertEqual(network.name, 'Error Handling Test')
        # Path should be empty since generation failed
        self.assertEqual(network.generated_files_path, '')
        # Parameters should NOT be populated if generation failed
        self.assertIsNone(network.output_size)
        self.assertEqual(network.layer_sizes, [])
    
    @patch('networks.views.generate_network')
    def test_create_view_mode3_passes_b_parameter(self, mock_generate):
        """Test that Mode 3 networks pass B parameter to generator"""
        mock_generate.return_value = '/path/to/network_1.zip'
        
        form_data = {
            'name': 'Mode 3 B Test',
            'description': 'Test B parameter',
            'mode': 3,
            'input_size': 4,
            'T': 16,
            'R': True,
            'B': 25
        }
        self.client.post(reverse('networks:create'), form_data)
        
        network = NetworkConfig.objects.first()
        # Verify B value is saved
        self.assertEqual(network.B, 25)
        
        # Verify generator was called with the network that has B=25
        called_network = mock_generate.call_args[0][0]
        self.assertEqual(called_network.B, 25)
    
    def test_create_view_post_invalid_data(self):
        """Test POST with invalid data doesn't create network"""
        form_data = {
            'description': 'Missing required fields',
            # Missing 'name', 'mode', 'input_size', 'T'
        }
        response = self.client.post(reverse('networks:create'), form_data)
        
        # No network should be created
        self.assertEqual(NetworkConfig.objects.count(), 0)
        # Should re-render form with errors
        self.assertTemplateUsed(response, 'networks/network_form.html')
        # Check that form has errors for required fields
        self.assertTrue(response.context['form'].errors)
    
    def test_create_view_saves_all_fields(self):
        """Test that all form fields are saved to the model"""
        form_data = {
            'name': 'Complete Test',
            'description': 'Complete field test',
            'mode': 2,
            'input_size': 16,
            'T': 32,
            'R': True,
            'B': 15
        }
        self.client.post(reverse('networks:create'), form_data)
        
        network = NetworkConfig.objects.first()
        self.assertEqual(network.name, 'Complete Test')
        self.assertEqual(network.description, 'Complete field test')
        self.assertEqual(network.mode, 2)
        self.assertEqual(network.input_size, 16)
        self.assertEqual(network.T, 32)
        self.assertTrue(network.R)
        self.assertEqual(network.B, 15)
    
    @patch('networks.views.generate_network')
    def test_create_view_b_uses_default_if_mode3(self, mock_generate):
        """Test that B defaults to 10 for Mode 3 if not explicitly provided"""
        mock_generate.return_value = '/path/to/network_1.zip'
        
        form_data = {
            'name': 'Default B Test',
            'description': 'Test default B',
            'mode': 3,
            'input_size': 4,
            'T': 16,
            'R': False,
            'B': 10  # Default value from form
        }
        self.client.post(reverse('networks:create'), form_data)
        
        network = NetworkConfig.objects.first()
        self.assertEqual(network.B, 10)


class NetworkDetailViewDownloadTestCase(TestCase):
    """Test suite for NetworkDetailView download functionality"""
    
    def setUp(self):
        """Set up test user, client, and network"""
        self.user = User.objects.create_user(
            username='testuser',
            email='test@example.com',
            password='testpass123'
        )
        self.client.login(username='testuser', password='testpass123')
        
        # Create a network with generated files
        self.network = NetworkConfig.objects.create(
            name='Download Test Network',
            description='Test download functionality',
            mode=1,
            input_size=8,
            T=16,
            R=False,
            generated_files_path='/path/to/network_1.zip'
        )
        
        # Create a network without generated files
        self.network_no_files = NetworkConfig.objects.create(
            name='No Files Network',
            description='Network without generated files',
            mode=1,
            input_size=8,
            T=16,
            R=False
        )
    
    def test_detail_view_get_shows_download_button(self):
        """Test that GET request shows download button when files exist"""
        response = self.client.get(reverse('networks:detail', kwargs={'pk': self.network.pk}))
        
        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'networks/network_detail.html')
        self.assertContains(response, 'Download Files')
        self.assertContains(response, '<form method="post"')
    
    def test_detail_view_get_shows_no_button_without_files(self):
        """Test that GET request hides download button when files don't exist"""
        response = self.client.get(reverse('networks:detail', kwargs={'pk': self.network_no_files.pk}))
        
        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'networks/network_detail.html')
        self.assertContains(response, 'Files Not Generated')
        self.assertNotContains(response, 'Download Files')
    
    @patch('builtins.open', create=True)
    def test_detail_view_post_downloads_file(self, mock_open):
        """Test that POST request downloads file when it exists"""
        from unittest.mock import MagicMock, mock_open as mock_open_helper
        
        # Mock file opening
        mock_file = MagicMock()
        mock_open.return_value = mock_file
        
        # Mock Path.exists() to return True
        with patch('networks.views.Path.exists', return_value=True):
            response = self.client.post(reverse('networks:detail', kwargs={'pk': self.network.pk}))
        
        # Should return FileResponse with status 200
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.get('Content-Type'), 'application/zip')
        self.assertEqual(response.get('Content-Disposition'), 
                        'attachment; filename="network_Download_Test_Network_1_8.zip"')
        # Verify file was opened
        mock_open.assert_called()
    
    def test_detail_view_post_without_files_returns_404(self):
        """Test that POST request returns Http404 when files not generated"""
        response = self.client.post(reverse('networks:detail', kwargs={'pk': self.network_no_files.pk}))
        
        # Should return 404
        self.assertEqual(response.status_code, 404)
    
    @patch('networks.views.Path.exists', return_value=False)
    def test_detail_view_post_missing_file_returns_404(self, mock_exists):
        """Test that POST request returns Http404 when file is missing from disk"""
        response = self.client.post(reverse('networks:detail', kwargs={'pk': self.network.pk}))
        
        # Should return 404
        self.assertEqual(response.status_code, 404)
    
    def test_detail_view_post_nonexistent_network_returns_404(self):
        """Test that POST request returns Http404 for nonexistent network"""
        response = self.client.post(reverse('networks:detail', kwargs={'pk': 999}))
        
        # Should return 404
        self.assertEqual(response.status_code, 404)
    
    def test_detail_view_post_filename_format(self):
        """Test that downloaded filename is properly formatted"""
        # Create network with spaces in name
        network_spaces = NetworkConfig.objects.create(
            name='Test Network With Spaces',
            mode=1,
            input_size=16,
            T=32,
            R=False,
            generated_files_path='/path/to/network.zip'
        )
        
        with patch('builtins.open', create=True):
            with patch('networks.views.Path.exists', return_value=True):
                response = self.client.post(
                    reverse('networks:detail', kwargs={'pk': network_spaces.pk})
                )
        
        # Check filename has spaces replaced with underscores
        content_disposition = response.get('Content-Disposition', '')
        self.assertIn('network_Test_Network_With_Spaces_1_16.zip', content_disposition)
        self.assertNotIn(' ', content_disposition.split('filename=')[1])
    
    def test_zip_contains_sv_files(self):
        """Test that downloaded ZIP contains SystemVerilog (.sv) files"""
        import zipfile
        import tempfile
        from pathlib import Path
        
        # Create a test ZIP with SV files
        with tempfile.TemporaryDirectory() as tmpdir:
            # Create mock SV files
            sv_dir = Path(tmpdir) / 'nn_8_8_16_1_1'
            sv_dir.mkdir(parents=True, exist_ok=True)
            
            sv_files = [
                'controller.sv',
                'datapath_gen_p3.sv',
                'datapath_gen_p3_relu.sv',
                'fc_8_8_16_1_1.sv',
                'memory.sv'
            ]
            
            for sv_file in sv_files:
                (sv_dir / sv_file).write_text(f"// Mock {sv_file}\nmodule test();\nendmodule")
            
            # Create a ZIP with these files
            zip_path = Path(tmpdir) / 'test_network.zip'
            with zipfile.ZipFile(zip_path, 'w') as zipf:
                for file in sv_dir.glob('*.sv'):
                    zipf.write(file, arcname=file.relative_to(sv_dir.parent))
                # Also add cost.txt
                cost_file = Path(tmpdir) / 'cost.txt'
                cost_file.write_text('1\n2\n3\n')
                zipf.write(cost_file, arcname='cost.txt')
            
            # Update network to point to this ZIP
            network = NetworkConfig.objects.create(
                name='SV Files Test',
                mode=1,
                input_size=8,
                T=16,
                R=False,
                generated_files_path=str(zip_path)
            )
            
            # Download and verify contents
            response = self.client.post(reverse('networks:detail', kwargs={'pk': network.pk}))
            
            self.assertEqual(response.status_code, 200)
            
            # Verify filename format matches network config
            content_disposition = response.get('Content-Disposition', '')
            expected_filename = f'network_SV_Files_Test_1_8.zip'
            self.assertIn(expected_filename, content_disposition)
            
            # Read the response as a ZIP file
            with tempfile.TemporaryDirectory() as extract_dir:
                # Save response content to file and read as ZIP
                downloaded_zip = Path(extract_dir) / 'downloaded.zip'
                downloaded_zip.write_bytes(b''.join(response.streaming_content))
                
                # Verify ZIP contents
                with zipfile.ZipFile(downloaded_zip, 'r') as zipf:
                    names = zipf.namelist()
                    
                    # Check that .sv files are present
                    sv_files_in_zip = [n for n in names if n.endswith('.sv')]
                    self.assertGreater(
                        len(sv_files_in_zip), 0,
                        f"No .sv files found in ZIP! Contents: {names}"
                    )
                    
                    # Verify specific SV files
                    expected_sv_files = [
                        'nn_8_8_16_1_1/controller.sv',
                        'nn_8_8_16_1_1/datapath_gen_p3.sv',
                        'nn_8_8_16_1_1/datapath_gen_p3_relu.sv',
                        'nn_8_8_16_1_1/fc_8_8_16_1_1.sv',
                        'nn_8_8_16_1_1/memory.sv'
                    ]
                    
                    for expected_file in expected_sv_files:
                        self.assertIn(
                            expected_file, names,
                            f"Expected file {expected_file} not found in ZIP! "
                            f"ZIP contains: {names}"
                        )
                    
                    # Also verify cost.txt is there
                    self.assertIn('cost.txt', names, "cost.txt not found in ZIP")
    
    def test_download_correct_files_for_network(self):
        """Test that download includes correct files for specific network instance"""
        import zipfile
        import tempfile
        from pathlib import Path
        
        # Create TWO test ZIPs with different content to simulate multiple networks
        with tempfile.TemporaryDirectory() as tmpdir:
            tmpdir = Path(tmpdir)
            
            # Network 1: nn_8_8_16_0_1 subdirectory
            sv_dir_1 = tmpdir / 'nn_8_8_16_0_1'
            sv_dir_1.mkdir(parents=True, exist_ok=True)
            (sv_dir_1 / 'controller.sv').write_text("// Network 1 Controller")
            (sv_dir_1 / 'memory.sv').write_text("// Network 1 Memory")
            
            # Network 2: nn_10_10_16_0_1 subdirectory (different dimensions)
            sv_dir_2 = tmpdir / 'nn_10_10_16_0_1'
            sv_dir_2.mkdir(parents=True, exist_ok=True)
            (sv_dir_2 / 'controller.sv').write_text("// Network 2 Controller")
            (sv_dir_2 / 'memory.sv').write_text("// Network 2 Memory")
            
            # Create ZIP for Network 1
            zip_path_1 = tmpdir / 'network_1_zip.zip'
            with zipfile.ZipFile(zip_path_1, 'w') as zipf:
                for file in sv_dir_1.glob('*.sv'):
                    zipf.write(file, arcname=file.relative_to(sv_dir_1.parent))
                (tmpdir / 'cost.txt').write_text('Network 1 Cost Data')
                zipf.write(tmpdir / 'cost.txt', arcname='cost.txt')
            
            # Create ZIP for Network 2
            zip_path_2 = tmpdir / 'network_2_zip.zip'
            with zipfile.ZipFile(zip_path_2, 'w') as zipf:
                for file in sv_dir_2.glob('*.sv'):
                    zipf.write(file, arcname=file.relative_to(sv_dir_2.parent))
                (tmpdir / 'cost2.txt').write_text('Network 2 Cost Data')
                zipf.write(tmpdir / 'cost2.txt', arcname='cost.txt')
            
            # Create two networks with different configs
            network_1 = NetworkConfig.objects.create(
                name='Network One',
                mode=1,
                input_size=8,
                T=16,
                R=False,
                generated_files_path=str(zip_path_1)
            )
            
            network_2 = NetworkConfig.objects.create(
                name='Network Two',
                mode=1,
                input_size=10,
                T=16,
                R=False,
                generated_files_path=str(zip_path_2)
            )
            
            # Download Network 1
            response_1 = self.client.post(
                reverse('networks:detail', kwargs={'pk': network_1.pk})
            )
            self.assertEqual(response_1.status_code, 200)
            
            # Verify Network 1 filename
            content_disp_1 = response_1.get('Content-Disposition', '')
            self.assertIn('network_Network_One_1_8.zip', content_disp_1)
            
            # Download Network 2
            response_2 = self.client.post(
                reverse('networks:detail', kwargs={'pk': network_2.pk})
            )
            self.assertEqual(response_2.status_code, 200)
            
            # Verify Network 2 filename
            content_disp_2 = response_2.get('Content-Disposition', '')
            self.assertIn('network_Network_Two_1_10.zip', content_disp_2)
            
            # Verify Network 1 ZIP contains correct subdirectory
            with tempfile.TemporaryDirectory() as extract_dir:
                extract_dir = Path(extract_dir)
                downloaded_zip_1 = extract_dir / 'network1.zip'
                downloaded_zip_1.write_bytes(b''.join(response_1.streaming_content))
                
                with zipfile.ZipFile(downloaded_zip_1, 'r') as zipf:
                    names_1 = zipf.namelist()
                    # Should contain Network 1's subdirectory
                    self.assertTrue(
                        any('nn_8_8_16_0_1' in name for name in names_1),
                        f"Network 1 subdirectory not found! ZIP contains: {names_1}"
                    )
                    # Should NOT contain Network 2's subdirectory
                    self.assertFalse(
                        any('nn_10_10_16_0_1' in name for name in names_1),
                        f"Network 2 subdirectory should NOT be in Network 1's ZIP! "
                        f"ZIP contains: {names_1}"
                    )
            
            # Verify Network 2 ZIP contains correct subdirectory
            with tempfile.TemporaryDirectory() as extract_dir:
                extract_dir = Path(extract_dir)
                downloaded_zip_2 = extract_dir / 'network2.zip'
                downloaded_zip_2.write_bytes(b''.join(response_2.streaming_content))
                
                with zipfile.ZipFile(downloaded_zip_2, 'r') as zipf:
                    names_2 = zipf.namelist()
                    # Should contain Network 2's subdirectory
                    self.assertTrue(
                        any('nn_10_10_16_0_1' in name for name in names_2),
                        f"Network 2 subdirectory not found! ZIP contains: {names_2}"
                    )
                    # Should NOT contain Network 1's subdirectory
                    self.assertFalse(
                        any('nn_8_8_16_0_1' in name for name in names_2),
                        f"Network 1 subdirectory should NOT be in Network 2's ZIP! "
                        f"ZIP contains: {names_2}"
                    )
    
    def test_zip_missing_sv_files_raises_error(self):
        """Test that generation fails if SV files are not created"""
        from pathlib import Path
        from generator.generator import GenerationError, create_output_zip
        from networks.models import NetworkConfig
        import tempfile
        
        # Create a directory with only cost.txt (no SV files)
        with tempfile.TemporaryDirectory() as tmpdir:
            output_dir = Path(tmpdir)
            
            # Create cost.txt but no .sv files
            (output_dir / 'cost.txt').write_text('1\n2\n3\n')
            
            # Create a mock network
            network = NetworkConfig(id=999, mode=1, input_size=8)
            
            # Should raise GenerationError because no .sv files found
            with self.assertRaises(GenerationError) as context:
                create_output_zip(network, output_dir)
            
            self.assertIn('No SystemVerilog', str(context.exception))


class NetworkCreateViewRateLimitTestCase(TestCase):
    """Test suite for NetworkCreateView rate limiting (3 requests per 60 seconds per IP)"""
    
    def setUp(self):
        """Set up test client and Redis connection"""
        self.client.defaults['REMOTE_ADDR'] = '192.168.1.100'  # Simulate client IP
        self.redis = get_redis_connection("default")
        # Clear rate limit keys before each test
        self.redis.flushdb()
    
    def tearDown(self):
        """Clean up Redis after each test"""
        self.redis.flushdb()
    
    def _get_network_form_data(self, name_suffix=''):
        """Helper to generate network form data"""
        return {
            'name': f'Test Network{name_suffix}',
            'description': 'Test network for rate limiting',
            'mode': 1,
            'input_size': 8,
            'output_size': 10,
            'T': 16,
            'R': False,
            'P': 1
        }
    
    @patch('networks.views.generate_network')
    def test_create_3_networks_succeeds(self, mock_generate):
        """Test that creating 3 networks within rate limit succeeds"""
        mock_generate.return_value = '/tmp/test.zip'
        
        # First 3 requests should succeed (200)
        for i in range(3):
            response = self.client.post(
                reverse('networks:create'),
                data=self._get_network_form_data(f'_{i}'),
                follow=True
            )
            self.assertIn(
                response.status_code,
                [200, 302],  # 200 for form display, 302 for redirect
                f"Request {i+1} failed with status {response.status_code}"
            )
    
    @patch('networks.views.generate_network')
    def test_create_4th_network_fails_rate_limit(self, mock_generate):
        """Test that 4th network creation within 60 seconds returns 429"""
        mock_generate.return_value = '/tmp/test.zip'
        
        # Make 3 successful requests
        for i in range(3):
            response = self.client.post(
                reverse('networks:create'),
                data=self._get_network_form_data(f'_{i}'),
                follow=False
            )
        
        # 4th request should be rate limited (429)
        response = self.client.post(
            reverse('networks:create'),
            data=self._get_network_form_data('_4'),
            follow=False
        )
        
        # Check for 429 status code
        self.assertEqual(
            response.status_code,
            429,
            f"Expected 429 rate limit, got {response.status_code}"
        )
    
    @patch('networks.views.generate_network')
    def test_rate_limit_headers_present(self, mock_generate):
        """Test that rate limit headers are present in response"""
        mock_generate.return_value = '/tmp/test.zip'
        
        response = self.client.post(
            reverse('networks:create'),
            data=self._get_network_form_data('_1'),
            follow=False
        )
        
        # Check for rate limit headers
        self.assertIn('X-RateLimit-Limit', response)
        self.assertIn('X-RateLimit-Remaining', response)
        self.assertIn('X-RateLimit-Reset', response)
        
        # Verify header values
        self.assertEqual(response['X-RateLimit-Limit'], '3')
        # Remaining should be 2 (1 request made, 3 total allowed)
        self.assertEqual(response['X-RateLimit-Remaining'], '2')
    
    @patch('networks.views.generate_network')
    def test_rate_limit_remaining_decreases(self, mock_generate):
        """Test that X-RateLimit-Remaining header decreases with each request"""
        mock_generate.return_value = '/tmp/test.zip'
        
        remaining_values = []
        
        for i in range(3):
            response = self.client.post(
                reverse('networks:create'),
                data=self._get_network_form_data(f'_{i}'),
                follow=False
            )
            remaining = int(response.get('X-RateLimit-Remaining', 0))
            remaining_values.append(remaining)
        
        # Should start at 2 (3 allowed - 1 request)
        # Then 1 (3 allowed - 2 requests)
        # Then 0 (3 allowed - 3 requests)
        self.assertEqual(remaining_values, [2, 1, 0])
    
    @patch('networks.views.generate_network')
    def test_rate_limit_message_on_429(self, mock_generate):
        """Test that 429 response includes helpful rate limit message"""
        mock_generate.return_value = '/tmp/test.zip'
        
        # Make 3 requests to hit the limit
        for i in range(3):
            self.client.post(
                reverse('networks:create'),
                data=self._get_network_form_data(f'_{i}'),
                follow=False
            )
        
        # 4th request should include message
        response = self.client.post(
            reverse('networks:create'),
            data=self._get_network_form_data('_4'),
            follow=False
        )
        
        self.assertEqual(response.status_code, 429)
        # Response should contain helpful message
        self.assertIn(
            b'Rate limit exceeded',
            response.content
        )
        self.assertIn(
            b'3 requests per 60 seconds',
            response.content
        )
    
    @patch('networks.views.generate_network')
    def test_different_ips_have_separate_limits(self, mock_generate):
        """Test that different IPs get separate rate limit counters"""
        mock_generate.return_value = '/tmp/test.zip'
        
        # Client 1: Make 3 requests with IP 192.168.1.100
        self.client.defaults['REMOTE_ADDR'] = '192.168.1.100'
        for i in range(3):
            response = self.client.post(
                reverse('networks:create'),
                data=self._get_network_form_data(f'_ip1_{i}'),
                follow=False
            )
        
        # Client 2: Make 3 requests with DIFFERENT IP - should succeed
        self.client.defaults['REMOTE_ADDR'] = '192.168.1.200'
        for i in range(3):
            response = self.client.post(
                reverse('networks:create'),
                data=self._get_network_form_data(f'_ip2_{i}'),
                follow=False
            )
            # Should be successful for the 2nd IP
            self.assertNotEqual(
                response.status_code,
                429,
                f"Request {i+1} from different IP was rate limited (should not be)"
            )
        
        # Client 2: 4th request should now be rate limited
        response = self.client.post(
            reverse('networks:create'),
            data=self._get_network_form_data('_ip2_4'),
            follow=False
        )
        self.assertEqual(response.status_code, 429)
