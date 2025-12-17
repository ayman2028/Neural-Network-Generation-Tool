from django.db import models

class NetworkConfig(models.Model):
    """
    Model to store neural network configuration parameters.
    Flexible design to support any number of layers (currently 1-3 via C++ tool, 
    but designed to scale to arbitrary layer counts in the future).
    
    FILE NAMING CONVENTION (from C++ code):
    ========================================
    Mode 1/2: Single Layer
    - Output directory: nn_{M}_{N}_{T}_{R}_{P}
    - Output file: fc_{M}_{N}_{T}_{R}_{P}.sv
    Example: nn_10_8_16_1_1/ and fc_10_8_16_1_1.sv
    
    Mode 3: Three-Layer Network
    - Output directory: nn_{N}_{M1}_{M2}_{M3}_{T}_{R}_{B}
    - Output file: net_{N}_{M1}_{M2}_{M3}_{T}_{R}_{B}.sv
    Example: nn_4_8_12_16_16_1_10/ and net_4_8_12_16_16_1_10.sv
    
    The parameters directly construct the filename, so no duplicate configs 
    can exist (perfect for caching).
    """
    
    MODE_CHOICES = [
        (1, 'Mode 1 - Single Unparallelized Layer'),
        (2, 'Mode 2 - Single Parallelized Layer'),
        (3, 'Mode 3 - Three-Layer Network'),
    ]
    
    # Metadata
    name = models.CharField(max_length=255, help_text="Name of the neural network")
    description = models.TextField(blank=True, help_text="Optional description of the network")
    mode = models.IntegerField(choices=MODE_CHOICES, help_text="Generation mode (1, 2, or 3)")
    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)
    generated_files_path = models.CharField(max_length=500, blank=True, help_text="Path to cached generated files")
    
    # Common parameters (for all modes)
    # Mode 1/2: input_size = N (input dimension)
    # Mode 3: input_size = N (input dimension)
    input_size = models.IntegerField(help_text="Input dimension (N)")
    
    # T = Bit width (e.g., 16 for 16-bit precision)
    # Used in all modes: fc_{M}_{N}_{T}_{R}_{P} or net_{N}_{M1}_{M2}_{M3}_{T}_{R}_{B}
    T = models.IntegerField(help_text="Bit width (e.g., 16 for 16-bit)")
    
    # R = ReLU activation (1 for yes, 0 for no)
    # Position 4 in Mode 1/2 filename, position 6 in Mode 3 filename
    R = models.BooleanField(default=False, help_text="ReLU activation (True/False)")
    
    # Mode 1 & 2 parameters
    # output_size = M (number of output neurons)
    # Position 1 in filename: fc_{M}_{N}_{T}_{R}_{P}
    output_size = models.IntegerField(null=True, blank=True, help_text="Output dimension (M) - Mode 1 & 2")
    
    # P = Parallelism factor (must be divisor of M)
    # Position 5 in Mode 1/2 filename: fc_{M}_{N}_{T}_{R}_{P}
    # Mode 1: P is typically 1, Mode 2: P > 1
    P = models.IntegerField(default=1, help_text="Parallelism factor (P) - Mode 1 & 2")
    
    # Mode 3 parameters - flexible layer sizes
    # layer_sizes stores [M1, M2, M3] for three-layer network
    # Positions 2-4 in Mode 3 filename: net_{N}_{M1}_{M2}_{M3}_{T}_{R}_{B}
    # Can be extended to support more layers in the future
    layer_sizes = models.JSONField(default=list, blank=True, help_text="List of layer output sizes [M1, M2, M3] - Mode 3")
    
    # B = Multiplier budget (optimization parameter for Mode 3)
    # Position 7 in Mode 3 filename: net_{N}_{M1}_{M2}_{M3}_{T}_{R}_{B}
    B = models.IntegerField(null=True, blank=True, help_text="Multiplier budget (B) - Mode 3")
    
    def __str__(self):
        return f"{self.name} (Mode {self.mode})"
    
    class Meta:
        verbose_name = "Network Configuration"
        verbose_name_plural = "Network Configurations"
        ordering = ['-created_at']
