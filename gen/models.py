from django.db import models
from django.contrib.auth.models import User

# Create your models here.

class NeuralNetworkConfig(models.Model):
    # Primary key (auto-incremented by Django by default)
    # id field is created automatically
    
    # Foreign key to Django's User model
    user = models.ForeignKey(User, on_delete=models.CASCADE, related_name='neural_networks')
    
    # Network architecture parameters
    # Defines the structure of the neural network: each element represents the number of neurons in each layer
    # e.g., [4, 8, 12, 16] = 4 input neurons, 8 neurons in layer 1, 12 in layer 2, 16 output neurons
    # Determines the network's capacity to learn complex patterns (more layers/neurons = more complex functions)
    layer_sizes = models.JSONField(help_text="List of layer sizes, e.g., [4, 8, 12, 16]")
    
    # Data precision parameters
    # Total bit-width for representing data values (inputs/outputs/activations)
    # Higher values = more precision but more hardware resources (registers, multipliers, memory)
    # e.g., 16 bits allows representation of values with finer granularity
    data_width = models.IntegerField(default=16)
    
    # Total bit-width for representing weight values
    # Weights are the learned parameters that define the network's behavior
    # More bits = more precise weights = potentially better accuracy but larger hardware footprint
    weight_width = models.IntegerField(default=16)
    
    # Number of fractional bits in fixed-point weight representation
    # e.g., weight_width=16, weight_frac=8 means 8 bits for integer, 8 bits for fraction
    # Determines the range vs. precision trade-off: more fractional bits = finer precision, smaller range
    weight_frac = models.IntegerField(default=8)
    
    # Parallelism settings
    # Number of multiply-accumulate (MAC) operations performed simultaneously in hardware
    # Higher parallelism = faster computation but more hardware resources (DSP blocks, logic)
    # e.g., parallelism=4 means 4 multiplications happen in parallel per clock cycle
    parallelism = models.IntegerField(default=1)
    
    # Activation function
    # Non-linear function applied after each layer's weighted sum
    # Enables the network to learn non-linear relationships in the data
    # ReLU (Rectified Linear Unit): fast, most common, outputs max(0, x)
    # Sigmoid: outputs values between 0 and 1, useful for probabilities
    # Tanh: outputs values between -1 and 1, zero-centered
    activation = models.CharField(max_length=20, default='relu', choices=[
        ('relu', 'ReLU'),
        ('sigmoid', 'Sigmoid'),
        ('tanh', 'Tanh'),
    ])
    
    # Generated files metadata
    # Path to the generated SystemVerilog files for this configuration
    file_path = models.CharField(max_length=255, blank=True, null=True)
    
    # Timestamps
    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)
    
    class Meta:
        ordering = ['-created_at']
    
    def __str__(self):
        return f"NN Config {self.id} by {self.user.username}"
