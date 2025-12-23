from django.urls import path
from . import views

app_name = 'networks'

urlpatterns = [
    # Dashboard (authenticated users only)
    path('dashboard/', views.dashboard, name='dashboard'),
    
    # List all networks
    path('', views.network_list, name='list'),
    
    # Network detail view
    #also handles file download via POST method.
    path('<int:pk>/', views.network_detail, name='detail'),
    
    # Create and generate network (POST) / View creation form (GET)
    path('create/', views.network_create, name='create'),
    
    # Download generated files as ZIP
    # path('<int:pk>/download/', views.network_download, name='download'),
    
    # Delete network
    # path('<int:pk>/delete/', views.network_delete, name='delete'),
]
