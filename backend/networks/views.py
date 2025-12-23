from django.shortcuts import render
from django.views.generic import ListView, DetailView, CreateView, TemplateView
from django.urls import reverse_lazy
from django.http import FileResponse, Http404
from django.contrib.auth.mixins import LoginRequiredMixin
from pathlib import Path
from .models import NetworkConfig
from .forms import NetworkConfigForm  # Import custom form for network creation
from generator.generator import generate_network, GenerationError


class NetworkListView(ListView):
    model = NetworkConfig
    template_name = 'networks/network_list.html'
    context_object_name = 'networks'


class NetworkDetailView(DetailView):
    model = NetworkConfig
    template_name = 'networks/network_detail.html'
    context_object_name = 'network'
    
    def post(self, request, *args, **kwargs):
        """
        Handle POST request to download generated network files.
        
        This method:
        1. Retrieves the network configuration
        2. Checks if files have been generated (generated_files_path is set)
        3. Verifies the file exists on disk
        4. Returns FileResponse for file download
        5. Handles errors gracefully with Http404
        
        Returns:
            FileResponse with ZIP file attachment
            
        Raises:
            Http404 if network not found, files not generated, or file missing
        """
        # Retrieve the network configuration
        network = self.get_object()
        
        # Check if files have been generated
        if not network.generated_files_path:
            raise Http404(
                f"No files have been generated for network '{network.name}'. "
                "Please create the network to generate files."
            )
        
        # Verify the file exists on disk
        file_path = Path(network.generated_files_path)
        if not file_path.exists():
            raise Http404(
                f"Generated files for network '{network.name}' are missing from disk. "
                f"The files may have been deleted. Please regenerate the network."
            )
        
        # Create a safe filename for download
        # Replace spaces with underscores to avoid issues
        safe_name = network.name.replace(' ', '_').replace('/', '_')
        download_filename = f"network_{safe_name}_{network.mode}_{network.input_size}.zip"
        
        # Return the file for download
        response = FileResponse(
            open(file_path, 'rb'),
            as_attachment=True,
            filename=download_filename,
            content_type='application/zip'
        )
        
        return response


class NetworkCreateView(CreateView):
    """
    View for creating new neural network configurations.
    
    This view:
    1. Displays a form for users to input network parameters (name, mode, dimensions, etc.)
    2. Saves the configuration to the database
    3. Automatically generates network files using the C++ generator
    4. Stores the path to generated files in the model for later download/reference
    """
    
    model = NetworkConfig
    template_name = 'networks/network_form.html'
    # Use custom form that includes the B (multiplier budget) parameter
    form_class = NetworkConfigForm
    
    def form_valid(self, form):
        """
        Override form_valid to add automatic file generation after save.
        
        This method:
        1. Saves the form data to the database (super().form_valid())
        2. Calls the C++ generator with the network configuration
        3. Populates mode-specific parameters (output_size, P, layer_sizes) after successful generation
        4. Stores the path to generated ZIP file in the model
        5. Handles generation errors gracefully (doesn't fail network creation)
        
        Args:
            form: The validated NetworkConfigForm instance
            
        Returns:
            The HTTP response from the parent class (typically a redirect)
        """
        response = super().form_valid(form)
        
        # Generate network files
        try:
            # Call generator.generate_network() which:
            # - Constructs the appropriate C++ command based on network mode
            # - Passes the B (multiplier budget) parameter for Mode 3
            # - Runs the C++ executable and returns path to generated ZIP file
            zip_file_path = generate_network(self.object)
            
            # Populate mode-specific parameters AFTER successful generation
            # Only populate if generation succeeds to avoid partial/incorrect data
            if self.object.mode in [1, 2]:
                # For Mode 1/2: set output_size (M) and parallelism (P)
                self.object.output_size = self.object.input_size  # M = N
                if self.object.mode == 1:
                    self.object.P = 1  # Mode 1 always has P=1 (unparallelized)
                # Mode 2 would use P from form, but we'll default to 1 for now
                
            elif self.object.mode == 3:
                # For Mode 3: set layer_sizes [M1, M2, M3]
                # Currently using input_size for M1 and M2, 1 for M3
                self.object.layer_sizes = [
                    self.object.input_size,  # M1
                    self.object.input_size,  # M2
                    1                         # M3
                ]
            
            # Store the generated file path in the model for later retrieval
            # This allows the DetailView to provide download links
            self.object.generated_files_path = str(zip_file_path)
            
            # Save all updated fields (generated path and mode-specific parameters)
            self.object.save(update_fields=[
                'generated_files_path', 
                'output_size', 
                'P', 
                'layer_sizes'
            ])
            
            print(f"Successfully generated network files: {zip_file_path}")
        except GenerationError as e:
            # Log the error but don't fail the network creation
            # This ensures the user's configuration is saved even if generation fails
            # (e.g., due to invalid parameters or C++ tool issues)
            print(f"Warning: Failed to generate network files: {e}")
        
        return response
    
    def get_success_url(self):
        return reverse_lazy('networks:detail', kwargs={'pk': self.object.pk})


class DashboardView(LoginRequiredMixin, TemplateView):
    """
    User dashboard showing their networks and quick actions.
    Only accessible to authenticated users.
    """
    template_name = 'networks/dashboard.html'
    login_url = 'login'
    
    def get_context_data(self, **kwargs):
        context = super().get_context_data(**kwargs)
        context['networks'] = NetworkConfig.objects.all()
        return context


# Create function-based view wrappers for URLs
network_list = NetworkListView.as_view()
network_detail = NetworkDetailView.as_view()
network_create = NetworkCreateView.as_view()
dashboard = DashboardView.as_view()
