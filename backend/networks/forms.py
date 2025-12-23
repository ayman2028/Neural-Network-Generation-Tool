from django import forms
from .models import NetworkConfig


class NetworkConfigForm(forms.ModelForm):
    """
    Form for creating and editing NetworkConfig instances.
    
    Extends Django's ModelForm to provide a user-friendly form with:
    - All necessary fields for network configuration
    - Custom widgets for Bootstrap styling
    - Help text for complex parameters like B (multiplier budget)
    - Default values for optimization parameters
    """
    
    class Meta:
        model = NetworkConfig
        # Fields to display in the form, including B (multiplier budget)
        fields = ['name', 'description', 'mode', 'input_size', 'T', 'R', 'B']
        widgets = {
            # TextInput for network name
            'name': forms.TextInput(attrs={'class': 'form-control'}),
            # Textarea for longer descriptions
            'description': forms.Textarea(attrs={'class': 'form-control', 'rows': 4}),
            # Dropdown select for network mode (1, 2, or 3)
            'mode': forms.Select(attrs={'class': 'form-control'}),
            # Number input for input dimension (N)
            'input_size': forms.NumberInput(attrs={'class': 'form-control'}),
            # Number input for bit width (T)
            'T': forms.NumberInput(attrs={'class': 'form-control'}),
            # Checkbox for ReLU activation (R)
            'R': forms.CheckboxInput(attrs={'class': 'form-check-input'}),
            # Number input for multiplier budget (B) - only used in Mode 3
            'B': forms.NumberInput(attrs={'class': 'form-control'}),
        }
        help_texts = {
            # Help text explains what B is and when it's used
            'B': 'Multiplier budget for optimization (recommended: 10 for Mode 3, ignored for Mode 1/2)',
        }
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # Set B initial value to 10 for new network configs
        # This provides a sensible default that users can override
        # B is an optimization parameter passed to the C++ generator for Mode 3
        if not self.instance.pk:
            self.fields['B'].initial = 10
