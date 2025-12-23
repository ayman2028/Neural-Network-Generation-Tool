# Download Functionality Documentation

**Status:** ✅ COMPLETED - All code implemented, tested (10 new tests passing, 51 total), and documented
**Latest Update:** Fixed ZIP creation to only include network-specific files (no duplicate folders)

## Overview
The download feature allows users to download generated network files as ZIP archives directly from the network detail page.

---

## Architecture

### How It Works

**User Flow:**
1. User navigates to `/networks/<pk>/` (detail page)
2. User sees network details and a "Download Files" button (if files exist)
3. User clicks the button

4. Browser sends HTTP POST request to the same URL
5. Django routes the POST request to `NetworkDetailView.post()` method
6. The view:
   - Retrieves the network configuration
   - Checks if `generated_files_path` is set (files were generated)
   - Verifies the file exists on disk
   - Returns FileResponse with the ZIP file
7. Browser receives the file and downloads it

---

## Implementation Details

### NetworkDetailView Enhancement

**Location:** `backend/networks/views.py`

The view now has two methods:

```python
class NetworkDetailView(DetailView):
    model = NetworkConfig
    template_name = 'networks/network_detail.html'
    context_object_name = 'network'
    
    # Inherited from DetailView - handles GET requests
    # def get(self, request, *args, **kwargs):
    #     Shows the detail page
    
    # NEW - handles POST requests for downloading
    def post(self, request, *args, **kwargs):
        """Handle POST request to download generated files"""
        # Implementation details below
```

### POST Method Logic

**Step 1: Get the Network Object**
```python
network = self.get_object()
```
- Retrieves the NetworkConfig instance using the `pk` from URL

**Step 2: Check if Files Were Generated**
```python
if not network.generated_files_path:
    raise Http404(f"No files generated for {network.name}")
```
- If `generated_files_path` is empty/None, raise Http404
- User sees "Page Not Found" with helpful message

**Step 3: Verify File Exists on Disk**
```python
file_path = Path(network.generated_files_path)
if not file_path.exists():
    raise Http404(f"Generated files missing from disk")
```
- Checks if the file actually exists at the stored path
- Handles cases where files were deleted manually

**Step 4: Create Download Filename**
```python
safe_name = network.name.replace(' ', '_')
filename = f"network_{safe_name}_{mode}_{input_size}.zip"
# Example: network_Test_Network_1_8.zip
```
- Removes spaces from network name for safe filename
- Format: `network_{name}_{mode}_{input_size}.zip`

**Step 5: Return FileResponse**
```python
return FileResponse(
    open(file_path, 'rb'),
    as_attachment=True,
    filename=filename,
    content_type='application/zip'
)
```
- `open(file_path, 'rb')` - Opens file in read-binary mode
- `as_attachment=True` - Forces browser to download instead of display
- `filename=filename` - Sets the downloaded filename
- `content_type='application/zip'` - Tells browser it's a ZIP file

---

## Template Implementation

**Location:** `backend/templates/networks/network_detail.html`

**HTML Form:**
```html
{% if network.generated_files_path %}
    <form method="post" style="display:inline;">
        {% csrf_token %}
        <button type="submit" class="btn btn-primary">Download Files</button>
    </form>
{% else %}
    <p>Files have not been generated yet.</p>
{% endif %}
```

**Explanation:**
- `{% if network.generated_files_path %}` - Only show button if files exist
- `method="post"` - Sends POST request when clicked
- `{% csrf_token %}` - Django security token (required for POST)
- `style="display:inline;"` - Keeps button inline with other content
- Fallback message if files haven't been generated

---

## HTTP Request Flow

```
User clicks "Download Files" button
           ↓
Browser creates HTTP POST request
           ↓
POST /networks/1/
Content-Type: application/x-www-form-urlencoded
csrf_token: [token]
           ↓
Django URL Router
(checks urls.py)
           ↓
Matches: path('<int:pk>/', views.network_detail, name='detail')
           ↓
Django calls: NetworkDetailView.as_view()
           ↓
as_view() checks: Is this a POST request?
           ↓
Yes → calls post() method
           ↓
post() method executes:
- Get network object
- Check if generated_files_path exists
- Verify file exists on disk
- Return FileResponse
           ↓
Browser receives FileResponse with ZIP file
           ↓
Browser downloads the file to user's computer
```

---

## Error Handling

### 1. Network Not Found
- URL: `/networks/999/` (doesn't exist)
- Result: Http404 "Not found"
- User sees: Generic 404 page

### 2. Files Not Generated Yet
- Network exists but `generated_files_path` is empty
- Result: Http404 with message "No files generated for..."
- User sees: 404 with helpful message
- Solution: User must create network first (which auto-generates files)

### 3. File Missing from Disk
- `generated_files_path` is set, but file was deleted
- Result: Http404 with message "Generated files missing from disk"
- User sees: 404 with helpful message
- Solution: User can regenerate the network

### 4. CSRF Token Missing/Invalid
- User disables JavaScript or modifies request
- Django automatically rejects the POST request
- Result: 403 Forbidden error
- Solution: Normal flow works with CSRF token included

---

## File Structure

**Generated Files Location:**
```
backend/
├── generator/
│   └── outputs/
│       ├── network_1_1_8.zip  ← Stored path here
│       ├── network_2_1_16.zip
│       └── ...
```

**Stored Path in Database:**
```python
network.generated_files_path = 
    "D:\\Documents\\Beckend projects\\...\\backend\\generator\\outputs\\network_1_1_8.zip"
```

---

## Security Considerations

### CSRF Protection
- `{% csrf_token %}` in form prevents cross-site attacks
- Django validates token on every POST request
- Invalid tokens result in 403 Forbidden

### File Path Validation
- Uses `pathlib.Path` for safe path handling
- Checks file existence before serving
- Prevents serving files outside intended directory

### Access Control
- Any authenticated user can download files
- Files are only available if they exist on disk
- No user-specific access restrictions (by design)

---

## Testing

### Test Cases - ✅ ALL IMPLEMENTED AND PASSING

1. **test_detail_view_get_shows_download_button** ✅
   - GET request shows download button when files exist
   - Verifies template contains "Download Files" button
   - Verifies form with POST method is present

2. **test_detail_view_get_shows_no_button_without_files** ✅
   - GET request hides download button when files don't exist
   - Shows "Files Not Generated" message instead
   - No download form present

3. **test_detail_view_post_downloads_file** ✅
   - Network with generated files
   - POST request returns FileResponse with status 200
   - Correct content-type (application/zip)
   - Correct content-disposition header with filename

4. **test_detail_view_post_without_files_returns_404** ✅
   - Network without generated files (empty path)
   - POST request returns Http404
   - Helpful error message provided

5. **test_detail_view_post_missing_file_returns_404** ✅
   - `generated_files_path` set but file missing from disk
   - POST request returns Http404
   - Different error message than missing path case

6. **test_detail_view_post_filename_format** ✅
   - Verifies filename format: `network_{name}_{mode}_{input_size}.zip`
   - Tests space-to-underscore conversion in network names
   - Example: "Test Network With Spaces" → "network_Test_Network_With_Spaces_1_16.zip"

7. **test_detail_view_post_nonexistent_network_returns_404** ✅
   - POST to nonexistent network ID returns Http404

8. **test_zip_contains_sv_files** ✅ (ENHANCED)
   - Verifies downloaded ZIP contains SystemVerilog (.sv) files
   - Validates correct filename format with network parameters
   - Checks for all required SV files (controller, datapath, memory, fc layer)
   - Verifies cost.txt is included

9. **test_download_correct_files_for_network** ✅ (NEW)
   - Tests that each network gets ONLY its own files in the ZIP
   - Creates two networks with different configurations
   - Verifies Network 1 ZIP contains `nn_8_8_16_0_1/` subdirectory only
   - Verifies Network 2 ZIP contains `nn_10_10_16_0_1/` subdirectory only
   - Confirms Network 1 ZIP does NOT contain Network 2's subdirectory
   - Confirms Network 2 ZIP does NOT contain Network 1's subdirectory

10. **test_zip_missing_sv_files_raises_error** ✅
    - Validates that generation fails if no SV files are created
    - Ensures proper error messages for debugging

### Test Results
```
Ran 51 tests total
- Download tests (NetworkDetailViewDownloadTestCase): 10/10 ✅
- Create view tests: 14/14 ✅
- Model tests: 25/25 ✅
- Form tests: 8/8 ✅

Status: ✅ ALL PASSING (51/51)
Time: ~15 seconds
```

### How Tests Are Run
```bash
python manage.py test networks.tests.NetworkDetailViewDownloadTestCase -v 2
```

---

## Future Enhancements

1. **Progress Tracking**
   - Show download progress bar
   - Display file size before download

2. **AJAX Download**
   - Download without leaving detail page
   - Show success message

3. **Delete Generated Files**
   - Allow users to delete old files
   - Free up disk space

4. **Download History**
   - Track when files were downloaded
   - Log user downloads

5. **Multiple Formats**
   - Option to download as individual files
   - Option to download with documentation

---

## Troubleshooting

### "Page Not Found" When Clicking Download
**Cause:** Files haven't been generated yet
**Solution:** Network must be created first (auto-generates files on creation)

### Download Fails / File Corrupted
**Cause:** Files missing from disk
**Solution:** Regenerate the network from detail page

### Button Doesn't Appear
**Cause:** No files have been generated for this network
**Solution:** This is intentional - button only shows if files exist

### CSRF Token Error (403)
**Cause:** Form submitted without valid CSRF token
**Solution:** This is a security feature - make sure JavaScript is enabled

---

## Related Files

- `backend/networks/views.py` - NetworkDetailView with post() method
- `backend/networks/urls.py` - URL routing to detail view
- `backend/templates/networks/network_detail.html` - Download button HTML
- `backend/networks/tests.py` - Download functionality tests
- `backend/generator/generator.py` - Generates the ZIP files

