
# ToDo

## House keeping
 * ~~Remove copy of Definitions.h~~
 * ~~Find EPOS~~
 * ~~Re-order data members to avoid gcc warnings~~
 * ~~Remove `bigss::` dependency~~ (see comment in code re. `time_now_ms`)
 * Add Qt widget
 * Add header in all files with license (MIT?)
 * Layout using cisst format?  4 spaces instead of 2
 * Rename `ai` to analog input
 * Rename data members and methods using CamelCase + prefix `m` for members.  Or do we prefer snake_case?
 * When two constructors initialize many data members, create Init method
 * Add `CISST_EXPORT`
 * Rename files to use prefix `mts` or `osa`

## Code
 * bigss::time_now_ms replaced by mtsComponentManager::GetInstance()->GetTimeServer().GetRelativeTime() * 1000.0.  We need a better approach for this.
 * It seems each motor uses it's own task.  If this is the case for all commands including motor control, each uses a USB command.  Is there a way to send a command to all motors?