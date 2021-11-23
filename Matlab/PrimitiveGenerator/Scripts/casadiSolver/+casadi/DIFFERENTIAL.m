function v = DIFFERENTIAL()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 122);
  end
  v = vInitialized;
end
