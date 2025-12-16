# Custom User Model Setup

## Overview
This document explains the custom user model implementation and the changes made to set up a custom user authentication system instead of using Django's default `User` model.

## Why a Custom User Model?
Django's default `User` model is generic and works for basic use cases. By creating a custom user model, we can:
- Add custom fields specific to our application
- Make certain fields required (like email)
- Customize the authentication behavior
- Have more control over user data

## Changes Made

### 1. Models (`accounts/models.py`)

We created a `CustomUser` model that extends Django's `AbstractUser`:

```python
from django.contrib.auth.models import AbstractUser

class CustomUser(AbstractUser):
    email = models.EmailField(unique=True, blank=False)
```

**Key Points:**
- `AbstractUser` provides all the default user functionality (username, password, first_name, last_name, etc.)
- We made the `email` field **required** (`blank=False`) and **unique** to ensure each user has a unique email address
- This model replaces Django's default `User` model

### 2. Forms (`accounts/forms.py`)

We created custom forms that work with our `CustomUser` model:

#### CustomUserCreationForm
```python
class CustomUserCreationForm(UserCreationForm):
    class Meta(UserCreationForm.Meta):
        model = CustomUser
        # Add 'email' to the fields tuple to make it a required field in the user creation form
        # This ensures users must provide an email when registering
        fields = UserCreationForm.Meta.fields + ('email',)
```

**What this does:**
- Extends Django's built-in `UserCreationForm`
- Sets the model to our `CustomUser`
- **Adds email to the form fields** - This makes email a required field when creating a user
- Users must provide email during registration

#### CustomUserChangeForm
```python
class CustomUserChangeForm(UserChangeForm):
    class Meta(UserChangeForm.Meta):
        model = CustomUser
        fields = UserChangeForm.Meta.fields
```

**What this does:**
- Extends Django's built-in `UserChangeForm`
- Used when updating an existing user's information
- Works with our `CustomUser` model

### 3. Admin Interface (`accounts/admin.py`)

We registered the custom user model with Django's admin:

```python
class CustomUserAdmin(UserAdmin):
    add_form = CustomUserCreationForm      # Form used when creating a new user
    form = CustomUserChangeForm            # Form used when editing an existing user
    model = CustomUser                     # The model to manage
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
```

**What this does:**
- Creates a custom admin class for managing users
- Uses our custom forms instead of Django's default forms
- `list_display` - Shows username and email columns in the user list view
- `fieldsets` - Organizes fields into sections when editing an existing user
- `add_fieldsets` - Organizes fields when creating a new user, with email included
- This appears in the Django admin dashboard at `/admin/`
- Administrators can create, edit, and delete users through the admin interface

### 4. Database Configuration (`config/settings.py`)

In the Django settings file, we specify the custom user model:

```python
AUTH_USER_MODEL = 'accounts.CustomUser'
```

**What this does:**
- Tells Django to use our `CustomUser` model instead of the default `User` model
- Must be set before running migrations
- All foreign key references to users should use this custom model

### 5. Migrations

After making these changes, we created migrations for the accounts app:

```bash
python manage.py makemigrations accounts
```

This created `accounts/migrations/0001_initial.py` which contains the database schema for:
- Creating the `CustomUser` model
- Setting up all the fields and constraints

To apply the migrations to the database:
```bash
python manage.py migrate accounts
```

## Summary of the Flow

1. **User Registration** → Uses `CustomUserCreationForm` → Requires email
2. **User Profile Updates** → Uses `CustomUserChangeForm` 
3. **Admin Panel** → Uses `CustomUserAdmin` class
4. **Database** → Stores all users in the `CustomUser` model

## Benefits of This Setup

✓ Email is required for all users
✓ Email must be unique (no duplicate emails)
✓ Easy to add more custom fields in the future
✓ Full control over user creation and management
✓ Works seamlessly with Django's authentication system
✓ Admin interface automatically supports our custom user model
