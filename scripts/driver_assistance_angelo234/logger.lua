local M = { enabled = false }

function M.setEnabled(value)
  M.enabled = value and true or false
end

function M.toggle()
  M.enabled = not M.enabled
  return M.enabled
end

function M.isEnabled()
  return M.enabled
end

function M.log(level, tag, msg)
  if M.enabled and log then
    log(level, tag, msg)
  end
end

return M
