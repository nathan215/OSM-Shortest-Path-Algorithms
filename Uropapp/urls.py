from django.urls import path
from . import views

urlpatterns = [
    path('', views.CalculatePathView.as_view(), name='home'),
    path('calculate_path/', views.CalculatePathView.as_view(), name='calculate_path'),
    path('show_path/', views.ShowPathsView.as_view(), name='show_path'),
]

