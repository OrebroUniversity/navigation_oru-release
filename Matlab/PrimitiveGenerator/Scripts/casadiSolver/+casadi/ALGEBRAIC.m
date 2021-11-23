function v = ALGEBRAIC()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 121);
  end
  v = vInitialized;
end
