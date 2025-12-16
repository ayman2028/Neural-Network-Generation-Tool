from django.contrib import admin
from django.contrib.auth.admin import UserAdmin
from .models import CustomUser
from .forms import CustomUserCreationForm, CustomUserChangeForm

# Register your models here.
#So we want to not have Django's default user admin interface, but instead use our custom forms for creating and changing users.

class CustomUserAdmin(UserAdmin):
    add_form = CustomUserCreationForm
    form = CustomUserChangeForm
    model = CustomUser
    # list_display: Controls which fields are displayed in the user list view in the admin interface
    # Shows username and email columns in the user list
    list_display = ['username', 'email']
    # fieldsets: Organizes fields into sections when viewing/editing an existing user
    # Uses the default UserAdmin fieldsets structure
    fieldsets = UserAdmin.fieldsets
    # add_fieldsets: Organizes fields into sections when creating a new user
    # Extends the default fieldsets to include the email field in the user creation form
    add_fieldsets = UserAdmin.add_fieldsets + (
        (None, {'fields': ('email',)}),
    )



admin.site.register(CustomUser, CustomUserAdmin)