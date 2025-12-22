from django.shortcuts import render
from django.views.generic import ListView, DetailView, CreateView
from django.urls import reverse_lazy
from .models import NetworkConfig


class NetworkListView(ListView):
    model = NetworkConfig
    template_name = 'networks/network_list.html'
    context_object_name = 'networks'


class NetworkDetailView(DetailView):
    model = NetworkConfig
    template_name = 'networks/network_detail.html'
    context_object_name = 'network'


class NetworkCreateView(CreateView):
    model = NetworkConfig
    template_name = 'networks/network_form.html'
    fields = ['name', 'description', 'mode', 'input_size', 'T', 'R']
    
    def get_success_url(self):
        return reverse_lazy('networks:detail', kwargs={'pk': self.object.pk})


# Create function-based view wrappers for URLs
network_list = NetworkListView.as_view()
network_detail = NetworkDetailView.as_view()
network_create = NetworkCreateView.as_view()
